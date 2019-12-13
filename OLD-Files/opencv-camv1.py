from http.server import BaseHTTPRequestHandler, HTTPServer
import time
import urllib.parse as urlparse
import math
import threading

# webszerver config

HOST_NAME = '0.0.0.0'
PORT_NUMBER = 9000
import cv2
# itt a raspberry csak csinál egy képet és lementi
def takeImage():
    cam = cv2.VideoCapture(0)

    img_counter = 0
    ret, frame = cam.read()
    img_name = "opencv_frame_{}.png".format(img_counter)
    cv2.imwrite(img_name, frame)
    print("{} written!".format(img_name))
    img_counter += 1
    (flag, encodedImage) = cv2.imencode(".jpg", frame)
    cam.release()
    print(encodedImage)
    yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')

    return encodedImage


def start_server(path, port=9000):
    class MyHandler(BaseHTTPRequestHandler):
        def do_HEAD(self):
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()

        def do_GET(self):
            self.respond({'status': 200})

        def http_temp(self, ajax, file="", isfile=True):
            s = ""
            if file == "":
                with open("html_page/basic2.html", "r") as f:
                    s = f.read()
            else:
                if isfile:
                    with open(file, "r") as f:
                        s = f.read()
                else:
                    s = file
            return str(s)

        def handle_http(self, status_code, path):
            self.send_response(status_code)

            print("PATH:" + path)
            if path.startswith("/ajax"):
                self.send_header('Content-type', 'multipart/x-mixed-replace')
                self.end_headers()

                content = takeImage()
            elif path == "/":
                self.send_header('Content-type', 'text/html')
                self.end_headers()

                content = self.http_temp(False)
            else:
                self.send_header('Content-type', 'text/html')
                self.end_headers()

                print("html_page" + path)
                content = self.http_temp(False, file="html_page" + path)

            return bytes(content, 'UTF-8')

        def respond(self, opts):
            response = self.handle_http(opts['status'], self.path)
            self.wfile.write(response)

    if __name__ == '__main__':
        server_class = HTTPServer
        httpd = server_class((HOST_NAME, PORT_NUMBER), MyHandler)
        print(time.asctime(), 'Server Starts - %s:%s' % (HOST_NAME, PORT_NUMBER))
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
        httpd.server_close()
        print(time.asctime(), 'Server Stops - %s:%s' % (HOST_NAME, PORT_NUMBER))


daemon = threading.Thread(name='daemon_server',
                          target=start_server,
                          args=('.', PORT_NUMBER))
daemon.setDaemon(True)  # Set as a daemon so it will be killed once the main thread is dead.
daemon.start()
while True:
    time.sleep(20)