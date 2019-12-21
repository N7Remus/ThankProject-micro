# USAGE
# python webstreaming.py --ip 0.0.0.0 --port 8000

# import the necessary packages
# a szükséges modulok importálása              //new
from imutils.video import VideoStream
from flask import Response
from flask import Flask
from flask import render_template
from flask import request
import threading
import argparse
import datetime
import imutils
import time
import cv2
import os.path
# initialize the output frame and a lock used to ensure thread-safe
# exchanges of the output frames (useful for multiple browsers/tabs
# are viewing tthe stream)
#új képkocka létrehozása és zárolása amíg az első folyamatban van              //new
outputFrame = None
lock = threading.Lock()

# initialize a flask object
#flask objektum létrehozása              //new
app = Flask(__name__)

# initialize the video stream and allow the camera sensor to
# warmup
# vs = VideoStream(usePiCamera=1).start()
#video közvetítés létrehozása és kamera szenzor engedélyezése              //new
vs = VideoStream(src=0).start()
time.sleep(2.0)

def root_dir():  # pragma: no cover
    return os.path.abspath(os.path.dirname(__file__))


def get_file(filename):  # pragma: no cover
    try:
        src = os.path.join(root_dir(), filename)
        # Figure out how flask returns static files
        # Tried:
        # - render_template
        # - send_file
        # This should not be so non-obvious
        return open(src).read()
    except IOError as exc:
        return str(exc)


@app.route("/")
def index():
    # return the rendered template
    # visszaadja a render sablont              //new
    #return render_template("index.html")
    #return render_template("joystick.html")
    return render_template("basic.html")


def detect_motion(frameCount):
    # grab global references to the video stream, output frame, and
    # lock variables
    # a globális referenciákat beleteszi a video közvetítésbe, kimeneti képkockába              //new
    # és a zároló változóba              //new
    global vs, outputFrame, lock


    # loop over frames from the video stream
    #ciklust készít a képkockákból              //new
    while True:
        # read the next frame from the video stream, resize it,
        #kiolvassa a következő képkockát a videóból, majd újra méretezi              //new
        # convert the frame to grayscale, and blur it
        #beszürkíti és elhomályosítja a képkockát              //new
        frame = vs.read()
        frame = imutils.resize(frame, width=400)

        with lock:
            outputFrame = frame.copy()


def generate():
    # grab global references to the output frame and lock variables
    #hozzáadja a globális referenciát a kimeneti képkockához és zárolja a változókat              //new
    global outputFrame, lock

    # loop over frames from the output stream
    #ciklust készít a kimeneti képkockából              //new
    while True:
        # wait until the lock is acquired
        #várakozik amíg a zároló meg nem kapta              //new
        with lock:
            # check if the output frame is available, otherwise skip
            #ellenőrzi hogy elérhető-e a kimeneti képkocka              //new
            #ellenkező esetben továbbugrik              //new
            # the iteration of the loop
            if outputFrame is None:
                continue

            # encode the frame in JPEG format
            #képkocka kódolása JPEG formátumban              //new
            (flag, encodedImage) = cv2.imencode(".jpg", outputFrame)

            # ensure the frame was successfully encoded
            #ellenőrzi hogy sikeres volt-e a kódolás              //new
            if not flag:
                continue

        # yield the output frame in the byte format
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encodedImage) + b'\r\n')


@app.route("/video_feed")
def video_feed():
    # return the response generated along with the specific media
    # type (mime type)
    return Response(generate(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/ajax/")
def ajax():
    # here we want to get the value of user (i.e. ?user=some-value)
    width = request.args.get('w')
    height = request.args.get('h')

    content = width
    mimetype="text/html"
    return Response(content, mimetype=mimetype)



@app.route('/<path:path>')
def get_resource(path):  # pragma: no cover
    mimetypes = {
        ".css": "text/css",
        ".html": "text/html",
        ".js": "application/javascript",
    }
    complete_path = os.path.join(root_dir(), path)
    ext = os.path.splitext(path)[1]
    mimetype = mimetypes.get(ext, "text/html")
    content = get_file(complete_path)
    return Response(content, mimetype=mimetype)



# check to see if this is the main thread of execution
if __name__ == '__main__':
    # construct the argument parser and parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--ip", type=str, required=True,
                    help="ip address of the device")
    ap.add_argument("-o", "--port", type=int, required=True,
                    help="ephemeral port number of the server (1024 to 65535)")
    ap.add_argument("-f", "--frame-count", type=int, default=32,
                    help="# of frames used to construct the background model")
    args = vars(ap.parse_args())

    # start a thread that will perform motion detection
    #mozgásérzékelés indítása              //new
    t = threading.Thread(target=detect_motion, args=(
        args["frame_count"],))
    t.daemon = True
    t.start()

    # start the flask app
    #flask alkalmazás indítása              //new
    app.run(host=args["ip"], port=args["port"], debug=True,
            threaded=True, use_reloader=False)

# release the video stream pointer
vs.stop()
