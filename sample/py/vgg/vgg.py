import numpy as np
import cv2

from keras.preprocessing import image
import keras.applications.vgg16 as vgg16


def test00(model):
    filename = "../../../data" + "/image/shiba00.bmp"

    img = image.load_img(filename, target_size=(224, 224))

    # PIL - > numpy.ndarray
    X = image.img_to_array(img)

    # (width, height, ch) -> (batch, rwidth, height, ch)
    X = np.expand_dims(X, axis=0)

    X = vgg16.preprocess_input(X)

    Y = model.predict(X)

    list= vgg16.decode_predictions(Y, top=4)
    for v in list[0]:
        print(v)


def test01(model):
    cap = cv2.VideoCapture(0)
    
    if cap.isOpened() is False:
        raise("could not find USB camera")

    cv2.namedWindow("img")

    key = 0

    # 27 = 'ESC' key
    while key != 27:

        ret, img = cap.read()

        if ret == False:
            continue

        rgb = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), (224, 224))
        
        cv2.imshow("img", img)
       
        key = cv2.waitKey(1)

        # uint8 - > float32
        X = rgb.astype(np.float32)

        # (width, height, ch) -> (batch, rwidth, height, ch)
        X = np.expand_dims(X, axis=0)

        X = vgg16.preprocess_input(X)

        Y = model.predict(X)

        list= vgg16.decode_predictions(Y, top=4)
        for v in list[0]:
            print(v)

    cv2.destroyAllWindows()

def _main():
    model = vgg16.VGG16(weights='imagenet', input_shape=(224, 224, 3))
    model.summary()

    test00(model)


if __name__ == "__main__":
    _main()