from app import app
from foscam import *

url = '192.168.1.22'
user = 'admin'
pwd = 'notreal'
cam = FoscamCamera(url=url, user=user, pwd=pwd)

def move_a_little(fos, go, stop):
    fos.move(go)
    time.sleep(2)
    fos.move(stop)

@app.route('/')
@app.route('/index')
def index():
    move_a_little(cam, cam.UP, cam.STOP_UP)
