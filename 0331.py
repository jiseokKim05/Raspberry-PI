# send_data.py
import serial
import time

# 포트 설정
port1 = "/dev/ttyS0"
port2 = "/dev/ttyUSB0"
baud_rate = 9600

try:
    # 시리얼 포트 초기화
    ser1 = serial.Serial(port1, baud_rate, timeout=1)
    ser2 = serial.Serial(port2, baud_rate, timeout=1)
    
    # 무한 루프 시작
    while True:
        # 첫 번째 포트로 데이터 전송
        data_to_send1 = "#S4321*"
        ser1.write(data_to_send1.encode())
        print(f"Sent to {port1}: {data_to_send1}")
        
        # 약간의 지연
        time.sleep(1)
        
        # 두 번째 포트로 데이터 전송
        data_to_send2 = "#S5678*"
        ser2.write(data_to_send2.encode())
        print(f"Sent to {port2}: {data_to_send2}")
        
        # 다음 반복 전 지연
        time.sleep(1)

except KeyboardInterrupt:
    # Ctrl+C로 종료시 처리
    print("\nProgram terminated by user")
    
finally:
    # 포트 닫기
    ser1.close()
    ser2.close()
    print("Serial ports closed")
