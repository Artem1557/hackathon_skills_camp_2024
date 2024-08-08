from ultralytics import YOLO
model = YOLO('Model_Cyber_AI_drone.pt')
while True:
    results = model.track(source='30.jpg', show = True)