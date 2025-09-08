import onnx
from onnxruntime.quantization import quantize_dynamic, QuantType

# Input and output paths
onnx_model_path = "yolov8n.onnx"
quantized_model_path = "yolov8n_quantized.onnx"

# Perform dynamic quantization
quantize_dynamic(
    onnx_model_path,
    quantized_model_path,
    weight_type=QuantType.QUInt8,  # Quantize weights to 8-bit unsigned integer
)

print(f"Quantized model saved to {quantized_model_path}")
