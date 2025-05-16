from robot_control import app
from display_ip import IPDisplay
import threading

if __name__ == '__main__':
    # Start the display in a separate thread
    try:
        display = IPDisplay()
        display_thread = threading.Thread(target=display.run, daemon=True)
        display_thread.start()
        print("IP Display started")
    except Exception as e:
        print(f"Could not initialize display: {e}")
        print("Continuing without display...")

    # Run the server on all interfaces
    print("Running server on port 5000...")
    app.run(host='0.0.0.0', port=5000, debug=True)