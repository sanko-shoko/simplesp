import os
import keras

from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Conv2D
from keras.layers import Dropout, Flatten, MaxPooling2D

# load mnist
(x_train, y_train), (x_test, y_test) = mnist.load_data()

# normalize
x_train = (x_train.astype('float32') - 127.5) / 127.5
x_test = (x_test.astype('float32') - 127.5) / 127.5

# one hot
y_train = keras.utils.to_categorical(y_train, 10)
y_test = keras.utils.to_categorical(y_test, 10)

# reshape
shape = (28, 28, 1)
x_train = x_train.reshape(-1, shape[0], shape[1], shape[2])
x_test = x_test.reshape(-1, shape[0], shape[1], shape[2])

print('x_train :', x_train.shape)
print('x_test  :', x_test.shape)
print('y_train :', y_train.shape)
print('y_test  :', y_test.shape)


# define model
def cnn_model(class_num, input_shape):
    model = Sequential()
    model.add(Conv2D(32, (3, 3), activation='relu', input_shape=input_shape))
    model.add(Conv2D(64, (3, 3), activation='relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Dropout(0.25))
    model.add(Flatten())
    model.add(Dense(128, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(class_num, activation='softmax'))
    return model

model = cnn_model(10, shape)

model.summary()

opt = keras.optimizers.Adadelta()
model.compile(loss=keras.losses.categorical_crossentropy, optimizer=opt, metrics=['accuracy'])


epochs = 10
batch_size = 128

param_folder = './param'

if not os.path.isdir(param_folder):
    os.makedirs(param_folder)

# call back (for save param)
cbk = keras.callbacks.ModelCheckpoint(filepath = os.path.join(param_folder, 'param{epoch:02d}.hdf5'))

# train
model.fit(x_train, y_train, batch_size, epochs=epochs, verbose = 1, callbacks = [cbk], validation_data = (x_test, y_test))
	 
# test
result = model.predict(x_test)
score = model.evaluate(x_test, y_test, verbose = 0)

print('test result:', result.argmax(axis=1))
print('test loss and accuracy:', score)
