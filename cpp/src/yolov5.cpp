#include "yolov5.hpp"


int Yolov5::get_width(int x, float gw, int divisor) {
    return int(ceil((x * gw) / divisor)) * divisor;
}

int Yolov5::get_depth(int x, float gd) {
    if (x == 1) return 1;
    int r = round(x * gd);
    if (x * gd - int(x * gd) == 0.5 && (int(x * gd) % 2) == 0) {
        --r;
    }
    return std::max<int>(r, 1);
}


std::vector<sl::uint2> Yolov5::cvt(const cv::Rect &bbox_in){
    std::vector<sl::uint2> bbox_out(4);
    bbox_out[0] = sl::uint2(bbox_in.x, bbox_in.y);
    bbox_out[1] = sl::uint2(bbox_in.x + bbox_in.width, bbox_in.y);
    bbox_out[2] = sl::uint2(bbox_in.x + bbox_in.width, bbox_in.y + bbox_in.height);
    bbox_out[3] = sl::uint2(bbox_in.x, bbox_in.y + bbox_in.height);
    return bbox_out;
}

bool Yolov5::initialize_engine(std::string& engine_name) {
	// deserialize the .engine and run inference
	std::ifstream file(engine_name,  std::ios::binary);
	if (!file.good()) {
		std::cout << "Error reading engine name!\n";
		return false;
	}
	char *trtModelStream = nullptr;
	size_t size = 0;
	file.seekg(0, file.end);
	size = file.tellg();
	file.seekg(0, file.beg);
	trtModelStream = new char[size];
	assert(trtModelStream);
	file.read(trtModelStream, size);
	file.close();

	//prepare input data
	runtime_ = createInferRuntime(gLogger);
	assert(runtime_ != nullptr);
	engine_ = runtime_->deserializeCudaEngine(trtModelStream, size);
	assert(engine_ != nullptr);
	context_ = engine_->createExecutionContext();
	context_->setName("Testing");
	assert(context_ != nullptr);
	delete[] trtModelStream;
	assert(engine_->getNbBindings() == 2);

	inputIndex_ = engine_->getBindingIndex(INPUT_BLOB_NAME);
	outputIndex_ = engine_->getBindingIndex(OUTPUT_BLOB_NAME);
	assert(inputIndex_ == 0);
	assert(outputIndex_ == 1);

	CUDA_CHECK(cudaMalloc(&buffers_[inputIndex_], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof (float)));
	CUDA_CHECK(cudaMalloc(&buffers_[outputIndex_], BATCH_SIZE * OUTPUT_SIZE * sizeof (float)));

	CUDA_CHECK(cudaStreamCreate(&stream_));
	assert(BATCH_SIZE == 1);
	return true;
}

void Yolov5::doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* input, float* output, int batchSize) {
    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof (float), cudaMemcpyHostToDevice, stream_));

    if(!context.enqueue(batchSize, buffers, stream_, nullptr)) {
	    std::cout << "error\n";
    }
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof (float), cudaMemcpyDeviceToHost, stream_));
    cudaStreamSynchronize(stream_);
}

bool Yolov5::prepare_inference(cv::Mat& img_cv_rgb) {
    if (img_cv_rgb.empty()) return false;
    cv::Mat pr_img = preprocess_img(img_cv_rgb, INPUT_W, INPUT_H);
    int i = 0;
    for (int row = 0; row < INPUT_H; ++row) {
        uchar* uc_pixel = pr_img.data + row * pr_img.step;
        for (int col = 0; col < INPUT_W; ++col) {
            data[batch_ * 3 * INPUT_H * INPUT_W + i] = (float) uc_pixel[2] / 255.0;
            data[batch_ * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float) uc_pixel[1] / 255.0;
            data[batch_ * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float) uc_pixel[0] / 255.0;
            uc_pixel += 3;
            ++i;
        }
    }
}
bool Yolov5::prepare_inference(sl::Mat img_sl, cv::Mat& img_cv_rgb) {
	cv::Mat left_cv_rgba = slMat2cvMat(img_sl);
	cv::cvtColor(left_cv_rgba, img_cv_rgb, cv::COLOR_BGRA2BGR);
    prepare_inference(img_cv_rgb);
}

void Yolov5::run_inference_and_convert_to_zed(cv::Mat& img_cv_rgb) {
	doInference(*context_, stream_, buffers_, data, prob, BATCH_SIZE);
	std::vector<std::vector<Yolo::Detection>> batch_res(BATCH_SIZE);
	auto& res = batch_res[batch_];
	nms(res, &prob[batch_ * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);

	for (auto &it : res) {
		sl::CustomBoxObjectData tmp;
		cv::Rect r = get_rect(img_cv_rgb, it.bbox);

		tmp.unique_object_id = sl::generate_unique_id();
		tmp.probability = it.conf;
		tmp.label = (int) it.class_id;
		tmp.bounding_box_2d = cvt(r);

		objects_in_.push_back(tmp);
	}
}

void Yolov5::run_inference(cv::Mat& img_cv_rgb) {
    monocular_objects_in_.clear();
    doInference(*context_, stream_, buffers_, data, prob, BATCH_SIZE);
    std::vector<std::vector<Yolo::Detection>> batch_res(BATCH_SIZE);
    auto& res = batch_res[batch_];
    nms(res, &prob[batch_ * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);
    for (auto &it : res) {
        cv::Rect r = get_rect(img_cv_rgb, it.bbox);
        tracked_object temp(r, it.class_id);
        monocular_objects_in_.push_back(temp);
    }
}

std::vector<sl::CustomBoxObjectData> Yolov5::get_custom_obj_data() {
	return objects_in_;
}

std::vector<tracked_object> Yolov5::get_monocular_obj_data() {
    return monocular_objects_in_;
}

void Yolov5::kill() {
	CUDA_CHECK(cudaFree(buffers_[inputIndex_]))
	CUDA_CHECK(cudaFree(buffers_[outputIndex_]))
	context_->destroy();
	engine_->destroy();
	runtime_->destroy();
}



