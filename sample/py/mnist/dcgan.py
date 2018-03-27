import os
import numpy as np
from PIL import Image

import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Conv2D, BatchNormalization
from keras.layers import Activation, Flatten, Dropout, UpSampling2D, MaxPooling2D, Reshape


# load mnist
(x_train, y_train), (x_test, y_test) = mnist.load_data()

# normalize
x_train = (x_train.astype('float32') - 127.5) / 127.5

# reshape
shape = (28, 28, 1)
x_train = x_train.reshape(-1, shape[0], shape[1], shape[2])

print('x_train :', x_train.shape)


# define model
def generator_model():
    model = Sequential()

    model.add(Dense(1024, input_shape = (100, )))
    model.add(Activation('tanh'))

    model.add(Dense(7 * 7 * 128))
    model.add(BatchNormalization())
    model.add(Activation('tanh'))

    model.add(Reshape((7, 7, 128), input_shape = (7 * 7 * 128,)))

    model.add(UpSampling2D(size = (2, 2)))
    model.add(Conv2D(64, (5, 5), padding='same'))
    model.add(Activation('tanh'))

    model.add(UpSampling2D(size = (2, 2)))
    model.add(Conv2D(1, (5, 5), padding = 'same'))
    model.add(Activation('tanh'))
    return model

def discriminator_model():
    model = Sequential()

    model.add(Conv2D(64, (5, 5), padding='same', input_shape=(28, 28, 1)))
    model.add(Activation('tanh'))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Conv2D(128, (5, 5)))
    model.add(Activation('tanh'))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Flatten())

    model.add(Dense(1024))
    model.add(Activation('tanh'))

    model.add(Dense(1))
    model.add(Activation('sigmoid'))
    return model


def combined_model(generator, discriminator):
    model = Sequential()
    model.add(generator)
    model.add(discriminator)
    return model

generator = generator_model()
generator.summary()

discriminator = discriminator_model()
discriminator.summary()

discriminator.trainable = False
combined = combined_model(generator, discriminator)
combined.summary()


opt = keras.optimizers.SGD(lr=0.0005, momentum=0.9, nesterov=True)

discriminator.trainable = True
discriminator.compile(loss='binary_crossentropy', optimizer=opt)

discriminator.trainable = False
combined.compile(loss='binary_crossentropy', optimizer=opt)


epochs = 100
batch_size = 128

param_folder = './param'

if not os.path.isdir(param_folder): 
    os.makedirs(param_folder)


for epoch in range(epochs):
    print('Epoch {}/{}'.format(epoch + 1, epochs))

    itmax = int(x_train.shape[0] / batch_size)
    progbar = keras.utils.generic_utils.Progbar(target = itmax)

    for i in range(itmax):

        # train discriminator
        x = x_train[i * batch_size : (i + 1) * batch_size]
        n = np.random.uniform(-1, 1, (batch_size, 100))
        g = generator.predict(n, verbose=0)
        y = [1] * batch_size + [0] * batch_size

        d_loss = discriminator.train_on_batch(np.concatenate((x, g)), y)

        # train generator
        n = np.random.uniform(-1, 1, (batch_size, 100))
        y = [1] * batch_size

        g_loss = combined.train_on_batch(n, y)

        progbar.add(1, values=[("d_loss", d_loss), ("g_loss", g_loss)])

        # save image
        if i % 20 == 0:
            tmp = [r.reshape(-1, 28) for r in np.split(g[:100], 10)]
            img = np.concatenate(tmp, axis = 1)
            img = (img * 127.5 + 127.5).astype(np.uint8)
            Image.fromarray(img).save("{}_{}.png".format(epoch, i))

    # save param
    generator.save_weights(os.path.join(param_folder, 'generator_{}.hdf5'.format(epoch)))
    discriminator.save_weights(os.path.join(param_folder, 'discriminator_{}.hdf5'.format(epoch)))
