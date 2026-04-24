# YOLO Models Directory

This directory is mounted into the container at /ros2_ws/models.

On first run, yolov8n.pt will be downloaded automatically by Ultralytics.
To use a custom-trained model, place it here and update vision_params.yaml:

  model_path: "/ros2_ws/models/your_custom_model.pt"

For OpenVINO-optimized models (.xml + .bin), export from Ultralytics:
  yolo export model=yolov8n.pt format=openvino
