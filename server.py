from robot_control import app, get_local_ip

if __name__ == '__main__':
    local_ip = get_local_ip()
    print(f"Server running at: http://{local_ip}:5000")
    print("You can also access it locally at: http://127.0.0.1:5000")
    # Run the server on all interfaces
    app.run(host='0.0.0.0', port=5000, debug=True) 