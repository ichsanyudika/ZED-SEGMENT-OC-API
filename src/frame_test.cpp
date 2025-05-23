#include <iostream>
#include <string>
#include <iomanip>
#include <chrono>
#include <thread>

// OpenCV includes untuk pengolahan gambar/video
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp> // Untuk menonaktifkan OpenCL jika perlu

// Include tambahan untuk fungsi video capture, kalibrasi dan stereo (asumsi dari library Stereolabs)
#include "videocapture.hpp"
#include "calibration.hpp"
#include "stereo.hpp"

int main(int argc, char *argv[])
{
    (void)argc; // Menghindari warning unused variable
    (void)argv;

    // Nonaktifkan OpenCL karena kadang bisa menyebabkan masalah pada beberapa hardware/driver
    cv::ocl::setUseOpenCL(false);

    // Set tingkat verbosity logging (INFO level)
    sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::INFO;

    // Konfigurasi parameter video, resolusi VGA dan FPS 30
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::VGA;  
    params.fps = sl_oc::video::FPS::FPS_30;
    params.verbose = verbose;

    // Inisialisasi video capture dengan parameter di atas
    sl_oc::video::VideoCapture cap(params);
    if (!cap.initializeVideo(-1)) { // -1 berarti default camera device
        std::cerr << "Cannot open camera video capture" << std::endl;
        return EXIT_FAILURE; // Exit jika gagal buka kamera
    }

    // Ambil serial number kamera yang terhubung
    int sn = cap.getSerialNumber();
    std::cout << "Connected to camera sn: " << sn << std::endl;

    // Mendownload file kalibrasi kamera dari server Stereolabs berdasarkan SN kamera
    std::string calibration_file;
    if (!sl_oc::tools::downloadCalibrationFile(sn, calibration_file)) {
        std::cerr << "Could not load calibration file from Stereolabs servers" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Calibration file found. Loading..." << std::endl;

    // Ambil ukuran frame dari kamera
    int w, h;
    cap.getFrameSize(w, h);

    // Mat untuk menyimpan peta remap kalibrasi stereo (x dan y) untuk kiri dan kanan
    cv::Mat map_left_x, map_left_y, map_right_x, map_right_y;
    // Mat kamera matriks intrinsic untuk kiri dan kanan
    cv::Mat cameraMatrix_left, cameraMatrix_right;
    double baseline = 0; // Jarak antar kamera

    // Inisialisasi kalibrasi stereo, buat peta remap dan dapatkan baseline
    sl_oc::tools::initCalibration(calibration_file, cv::Size(w / 2, h),
                                  map_left_x, map_left_y, map_right_x, map_right_y,
                                  cameraMatrix_left, cameraMatrix_right, &baseline);

    // Ambil focal length dan pusat proyeksi dari matriks kalibrasi kiri
    double fx = cameraMatrix_left.at<double>(0, 0);
    double fy = cameraMatrix_left.at<double>(1, 1);
    double cx = cameraMatrix_left.at<double>(0, 2);
    double cy = cameraMatrix_left.at<double>(1, 2);

    std::cout << "Camera Matrix L: \n" << cameraMatrix_left << std::endl;
    std::cout << "Baseline: " << baseline << " mm" << std::endl;

    // Variabel untuk frame dan hasil olahan
    cv::Mat frameYUV, frameBGR, left_raw, right_raw, left_rect, right_rect;
    cv::Mat left_disp, left_disp_float;

    // Load parameter StereoSGBM (disparity map) dari file jika ada, atau simpan default
    sl_oc::tools::StereoSgbmPar stereoPar;
    if (!stereoPar.load()) stereoPar.save();

    // Buat objek StereoSGBM dengan parameter yang sudah di-load
    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(
        stereoPar.minDisparity,
        stereoPar.numDisparities,
        stereoPar.blockSize);

    // Set parameter lain untuk StereoSGBM sesuai konfigurasi
    left_matcher->setMinDisparity(stereoPar.minDisparity);
    left_matcher->setNumDisparities(stereoPar.numDisparities);
    left_matcher->setBlockSize(stereoPar.blockSize);
    left_matcher->setP1(stereoPar.P1);
    left_matcher->setP2(stereoPar.P2);
    left_matcher->setDisp12MaxDiff(stereoPar.disp12MaxDiff);
    left_matcher->setMode(stereoPar.mode);
    left_matcher->setPreFilterCap(stereoPar.preFilterCap);
    left_matcher->setUniquenessRatio(stereoPar.uniquenessRatio);
    left_matcher->setSpeckleWindowSize(stereoPar.speckleWindowSize);
    left_matcher->setSpeckleRange(stereoPar.speckleRange);

    std::cout << "Starting real-time depth measurement...\nPress 'q' to exit" << std::endl;

    // Variabel untuk menghitung FPS dan timestamp frame
    uint64_t last_ts = 0;
    int frame_count = 0;
    auto t_start = std::chrono::high_resolution_clock::now();
    float fps = 0;

    // Loop utama pemrosesan frame video
    while (true) {
        // Ambil frame terbaru dari kamera
        const sl_oc::video::Frame frame = cap.getLastFrame();

        // Cek apakah data frame ada dan timestamp berbeda dengan frame sebelumnya
        if (frame.data != nullptr && frame.timestamp != last_ts) {
            last_ts = frame.timestamp;
            frame_count++;

            // Hitung FPS setiap 1 detik
            auto t_now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(t_now - t_start).count();
            if (elapsed > 1.0) {
                fps = frame_count / elapsed;
                frame_count = 0;
                t_start = t_now;
            }

            // Buat cv::Mat dari data frame YUV (format YUYV 8UC2) lalu clone agar tidak berubah saat frame baru datang
            frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data).clone();
            // Konversi dari YUV ke BGR untuk OpenCV agar bisa diproses/dilihat
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

            // Potong frame menjadi dua bagian: kiri dan kanan (kamera stereo)
            right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows)).clone();
            left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows)).clone();
            // Clone right image untuk nanti digambari hasil depth
            cv::Mat right_raw_with_depth = right_raw.clone();

            // Remap/pemulusan gambar kiri dan kanan berdasarkan kalibrasi
            cv::remap(left_raw, left_rect, map_left_x, map_left_y, cv::INTER_LINEAR);
            cv::remap(right_raw, right_rect, map_right_x, map_right_y, cv::INTER_LINEAR);

            // Hitung disparity map (perbedaan posisi pixel kiri dan kanan)
            left_matcher->compute(left_rect, right_rect, left_disp);
            // Konversi hasil disparity ke float (nilai asli disparity dibagi 16)
            left_disp.convertTo(left_disp_float, CV_32F, 1.0 / 16.0);

            // Buat matriks kosong untuk depth map
            cv::Mat depth_map = cv::Mat(left_disp_float.size(), CV_32F, cv::Scalar(0));
            // Hitung fx * baseline (parameter penting untuk konversi disparity ke jarak)
            double fx_baseline = fx * baseline;

            // Loop tiap pixel untuk hitung depth dari disparity
            for (int y = 0; y < left_disp_float.rows; y++) {
                for (int x = 0; x < left_disp_float.cols; x++) {
                    float disp = left_disp_float.at<float>(y, x);
                    // Jika disparity valid (lebih dari 1 dan kurang dari max disparity), hitung depth
                    if (disp > 1.0 && disp < stereoPar.numDisparities)
                        depth_map.at<float>(y, x) = static_cast<float>(fx_baseline / disp);
                }
            }

            // Median blur untuk mengurangi noise pada depth map
            cv::medianBlur(depth_map, depth_map, 5);

            // Tentukan ROI (region of interest) di tengah frame untuk hitung depth rata-rata
            int center_x = depth_map.cols / 2;
            int center_y = depth_map.rows / 2;
            cv::Rect roi(center_x - 3, center_y - 3, 7, 7);
            roi &= cv::Rect(0, 0, depth_map.cols, depth_map.rows); // Pastikan ROI tidak keluar frame

            // Ambil bagian tengah depth map, hitung rata-rata depth
            cv::Mat center_region = depth_map(roi);
            cv::Scalar avg_depth = cv::mean(center_region);
            float center_depth = static_cast<float>(avg_depth[0]);
            // Jika depth tidak valid, set ke 0
            if (center_depth <= 0 || center_depth > 10000) center_depth = 0;

            // Print depth (dalam meter, karena baseline mm dibagi 1000)
            std::cout << "Depth: " << std::fixed << std::setprecision(2) << center_depth / 1000.0 << " m" << std::endl;

            // Gambar lingkaran merah di titik tengah untuk menandai pengukuran depth
            cv::circle(right_raw_with_depth, cv::Point(center_x, center_y), 6, cv::Scalar(0, 0, 255), -1);
            // Tampilkan teks informasi depth pada frame kanan
            std::stringstream depth_ss;
            depth_ss << "Depth: " << std::fixed << std::setprecision(2) << center_depth / 1000.0 << " m";
            cv::putText(right_raw_with_depth, depth_ss.str(), cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            // Tampilkan FPS pada frame
            std::stringstream fps_ss;
            fps_ss << "FPS: " << std::fixed << std::setprecision(1) << fps;
            cv::putText(right_raw_with_depth, fps_ss.str(), cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            // Optional: visualisasi depth map dengan colormap JET (warna)
            cv::Mat depth_vis;
            depth_map.convertTo(depth_vis, CV_8UC1, 255.0 / 5000.0);
            cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
            // cv::imshow("Depth Map", depth_vis);

            // Tampilkan hasil gambar kanan dengan depth dan info
            cv::imshow("Right Image (Raw)", right_raw_with_depth);
        }

        // Tunggu input keyboard 1ms, keluar jika tekan q, Q, atau ESC
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) break;
    }

    std::cout << "Exiting program..." << std::endl;
    return EXIT_SUCCESS;
}
