import cv2
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np
from PIL import Image



def _preprocess_trt(img, shape=(1024, 1024)):
    """TRT SSD 推理前的数据前处理"""
    # print(f"img.shape: {str(img.shape)}")
    # print(f"shape: {shape}")
    img = cv2.resize(img, shape)
    img = img.transpose((2, 0, 1)).astype(np.float32)
    return img
###这里的 shape 需要根据自己的模型文件调整。可以参考下面的内容查看

def _postprocess_trt(img, output, conf_th, output_layout):
    """TRT SSD 推理后的结果的数据处理步骤."""
    img_h, img_w, _ = img.shape
    boxes, confs, clss = [], [], []
    for prefix in range(0, len(output), output_layout):
        index = int(output[prefix+0])
        conf = float(output[prefix+2])
        if conf < conf_th:
           continue
        x1 = int(output[prefix+3] * img_w)
        y1 = int(output[prefix+4] * img_h)
        x2 = int(output[prefix+5] * img_w)
        y2 = int(output[prefix+6] * img_h)
        cls = int(output[prefix+1])
        boxes.append((x1, y1, x2, y2))
        confs.append(conf)
        clss.append(cls)
    return boxes, confs, clss # 返回标框坐标、置信度、类别

class TrtSSD(object):
# 加载自定义组建，如果 TRT 版本小于 7.0 需要额外生成 flattenconcat 自定义组件库
    def _load_plugins(self):
        if trt.__version__[0] < '7':
            # !没有该文件
            ctypes.CDLL("ssd/libflattenconcat.so")
        trt.init_libnvinfer_plugins(self.trt_logger, '')
    #加载通过 Transfer Learning Toolkit 生成的推理引擎
    def _load_engine(self):
        # TRTbin = 'ssd/TRT_%s.bin' % self.model #请根据实际状况自行修改
        TRT = self.model
        with open(TRT, 'rb') as f, trt.Runtime(self.trt_logger) as runtime:
            return runtime.deserialize_cuda_engine(f.read())
    #通过加载的引擎，生成可执行的上下文
    def _create_context(self):
        for binding in self.engine:
            size = trt.volume(self.engine.get_binding_shape(binding))
            ##注意：这里的 host_mem 需要使用 pagelocked memory，以免内存被释放
            host_mem = cuda.pagelocked_empty(size, np.float32)
            cuda_mem = cuda.mem_alloc(host_mem.nbytes)
            self.bindings.append(int(cuda_mem))
            if self.engine.binding_is_input(binding):
                self.host_inputs.append(host_mem)
                self.cuda_inputs.append(cuda_mem)
            else:
                self.host_outputs.append(host_mem)
                self.cuda_outputs.append(cuda_mem)
        return self.engine.create_execution_context()
    # 初始化引擎
    def __init__(self, model, input_shape, output_layout=7):
        self.cfx = cuda.Device(0).make_context()  #2. trt engine创建前首先初始化cuda上下文
        self.model = model
        self.input_shape = input_shape
        self.output_layout = output_layout
        self.trt_logger = trt.Logger(trt.Logger.INFO)
        self._load_plugins()
        self.engine = self._load_engine()
        self.host_inputs = []
        self.cuda_inputs = []
        self.host_outputs = []
        self.cuda_outputs = []
        self.bindings = []
        self.stream = cuda.Stream()
        self.context = self._create_context()
    # 释放引擎，释放 GPU 显存，释放 CUDA 流
    def __del__(self):
        del self.stream
        del self.cuda_outputs
        del self.cuda_inputs
        self.cfx.detach() # 2. 实例释放时需要detech cuda上下文
    # 利用生成的可执行上下文执行推理
    def detect(self, img, conf_th=0.3):
        img_resized = _preprocess_trt(img, self.input_shape)
        np.copyto(self.host_inputs[0], img_resized.ravel())
        self.cfx.push()  # 3. 推理前执行cfx.push()
        # 将处理好的图片从 CPU 内存中复制到 GPU 显存
        cuda.memcpy_htod_async(
        self.cuda_inputs[0], self.host_inputs[0], self.stream)
        # 开始执行推理任务
        self.context.execute_async(
            batch_size=1,
            bindings=self.bindings,
            stream_handle=self.stream.handle)
        # 将推理结果输出从 GPU 显存复制到 CPU 内存
        cuda.memcpy_dtoh_async(
            self.host_outputs[1], self.cuda_outputs[1], self.stream)
        cuda.memcpy_dtoh_async(
            self.host_outputs[0], self.cuda_outputs[0], self.stream)
        self.stream.synchronize()
        output = self.host_outputs[0]
        output = _postprocess_trt(img, output, conf_th,self.output_layout)
        self.cfx.pop()  # 3. 推理后执行cfx.pop()
        return output
    
def detect_one(img, trt_ssd, conf_th):
##开始检测，并将结果写到 result.jpg 中
    boxes, confs, clss = trt_ssd.detect(img, conf_th)
    print("boxes: "+str(boxes))
    print("clss: "+str(clss))
    print("confs: "+str(confs))
    # img = vis.draw_bboxes(img, boxes, confs, clss)
    for (box, conf, cls) in zip(boxes, confs, clss):
    # 绘制边界框，box 格式假设为 [x_min, y_min, x_max, y_max]
        cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 2)
        # 在边界框旁边添加文本（类别和置信度）
        text = f'{cls}: {conf:.2f}'
        cv2.putText(img, text, (int(box[0]), int(box[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    cv2.imwrite("result.jpg",img)
def main_one():
    filename = "/home/elephant/xzc_code/tensorrt_demos/test_picture/662bd18fd33c8c356bbfc33f3e8bb93.jpg"
    model_name ="/home/elephant/dev/team1/model/ssd_resnet18_epoch_070.engine"
    INPUT_HW = (1280, 960)
    img = cv2.imread(filename)
    trt_ssd = TrtSSD(model_name, INPUT_HW)
    input()
    print("start detection!")
    detect_one(img, trt_ssd, conf_th=0.35)
    print("finish!")
    
# from IPython.display import Image
if __name__ == "__main__":
    main_one()
# Image("result.jpg")