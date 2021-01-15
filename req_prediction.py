#!/usr/bin/env python
# coding: utf-8

from glob import glob
import random

import keras
from keras.models import Sequential
from keras.layers import LSTM, Dense, Dropout
import numpy as np
import matplotlib.pyplot as plt

from keras.wrappers.scikit_learn import KerasRegressor
from keras.callbacks import EarlyStopping
from sklearn.model_selection import GridSearchCV


initializer = keras.initializers.RandomUniform(
    minval=-0.05, maxval=0.05, seed=1234)


multiplier = 32 * 1024
column = 7


def load_data(user):
    with open(user) as u:
        lines = [float(i.split(",")[column]) * multiplier
                 for i in u.readlines()]
    return np.array(lines)


def create_sequences(user_data, seq_size=20):
    sequences = []
    targets = []

    for index, point in enumerate(user_data):
        seq = []
        for i in range(seq_size):
            try:
                seq.append(user_data[index + i].tolist())
            except IndexError:
                break
        sequences.append(np.array(seq))

    output = np.empty((len(sequences), seq_size, 1))
    for i, v in enumerate(sequences):
        try:
            output[i] = np.array(v)
        except ValueError:
            pass
    return output[:, :seq_size-1], output[:, -1]


def create_model(learning_rates, n_layers, n_lstm_cells,
                 n_dense_neurons, droput_rate):
    model = Sequential()
    model.add(LSTM(n_lstm_cells, input_shape=(19, 1), dropout=droput_rate,
                   recurrent_dropout=droput_rate, kernel_initializer=initializer))
#     model.add(Dense(n_dense_neurons, activation="relu", input_shape=(9, 2)))
#     model.add(keras.layers.LeakyReLU())
    for i in range(n_layers):
        model.add(Dense(n_dense_neurons, kernel_initializer=initializer,
                        bias_initializer=initializer))
        model.add(keras.layers.LeakyReLU())
        model.add(Dropout(droput_rate))

    model.add(Dense(1, kernel_initializer=initializer,
                    bias_initializer=initializer))
    model.add(keras.layers.LeakyReLU())

    print(model.summary())

    model.compile(loss="mean_squared_error", metrics=[
                  "mean_squared_error"], optimizer=keras.optimizers.SGD(lr=learning_rates, clipvalue=0.5))

    return model


def main():

    users = glob("./user_requests/*")

    seq_size = 20

    for user in users:
        print(f"opening user {user}")
        data = load_data(user)
        seq_size = 20  # predict next position based on last 10 data points
        train, target = create_sequences(data, seq_size)

        es = EarlyStopping(patience=10, restore_best_weights=True)
        model = create_model(0.1, 2, 4, 50, 0.01)
        model.fit(train, target, epochs=20, verbose=1,
                  batch_size=1, validation_split=0.2, callbacks=[])
        model.save(f"users_koln/{user}.h5")

        for layer in model.layers:
            print(layer.get_weights())


main()
