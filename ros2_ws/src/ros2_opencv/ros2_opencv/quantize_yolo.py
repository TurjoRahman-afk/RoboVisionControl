from ultralytics import YOLO

def export_and_quantize():
    # Load YOLOv8 model
    model = YOLO("yolov8n.pt")
    
    # Step 1: Export to ONNX format (required for quantization)
    model.export(format="onnx", dynamic=True, simplify=True)
    
    print("Model exported to ONNX. Now use ONNX Runtime for quantization.")

if __name__ == "__main__":
    export_and_quantize()
