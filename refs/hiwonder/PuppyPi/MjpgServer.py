#!/usr/bin/python3
'''
    Author: Igor Maculan - n3wtron@gmail.com
    A Simple mjpg stream http server
'''
import cv2
import time
import threading
from socketserver import ThreadingMixIn
from http.server import BaseHTTPRequestHandler, HTTPServer,ThreadingHTTPServer

img_show = None
quality = (int(cv2.IMWRITE_JPEG_QUALITY), 70)

class MJPG_Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        global img_show
        if self.path == '/?action=snapshot':
            if img_show is not None:
                try:
                    l_quality = (int(cv2.IMWRITE_JPEG_QUALITY), 100)
                    ret, jpg = cv2.imencode('.jpg', img_show, l_quality) 
                    jpg_bytes = jpg.tobytes()
                    self.send_response(200)
                    self.send_header('Content-type','image/jpeg')
                    self.send_header('Content-length', len(jpg_bytes))
                    self.end_headers()
                    self.wfile.write(jpg_bytes)
                except Exception as e:
                    print(e)
        else:
            img_show = None
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--boundarydonotcross')
            self.end_headers()
            while True:
                try:
                    if img_show is not None:
                        ret, jpg = cv2.imencode('.jpg', img_show, quality)
                        jpg_bytes = jpg.tobytes()
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', len(jpg_bytes))
                        #self.send_header('X-Timestamp:', time.time())
                        self.wfile.write('--boundarydonotcross\r\n'.encode())
                        self.end_headers()
                        self.wfile.write(jpg_bytes)
                    time.sleep(0.05)
                except Exception as e:
                    print(e,"EE")
                    break

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""

def startMjpgServer():
    try:
        server = ThreadedHTTPServer(('', 8080), MJPG_Handler)
        print("server started")
        server.serve_forever()
    except KeyboardInterrupt:
        pass