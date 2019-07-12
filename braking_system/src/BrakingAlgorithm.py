#! /usr/bin/env python
import numpy as np
import pandas as pd


class BrakingAlgorithm:
    weightLayer1 = pd.read_excel('layer1Weights.xlsx')
    weightLayer2 = pd.read_excel('layer2Weights.xlsx')
    weightLayer3 = pd.read_excel('layer3Weights.xlsx')
    biasLayer1 = pd.read_excel('layer1Bias.xlsx')
    biasLayer2 = pd.read_excel('layer2Bias.xlsx')
    biasLayer3 = 0.0954225063323975

    def algo(self, input):
        firstLayer = np.matmul(input, np.asarray(self.weightLayer1.transpose())) + np.asarray(self.biasLayer1)
        secondLayer = np.matmul(np.maximum(firstLayer, 0), np.asarray(self.weightLayer2.transpose())) + np.asarray(
            self.biasLayer2)
        thirdLayer = np.matmul(np.maximum(secondLayer, 0), np.asarray(self.weightLayer3.transpose())) + np.asarray(
            self.biasLayer3)
        return self.hard_sigmoid(thirdLayer)

    def normalize(self, input):
        input = np.hstack(input[0] / 120, input[1] / 40)
        return input

    def hard_sigmoid(self, x):
        if x[0][0] < -2.5:
            return np.zeros([1, 2])
        if x[0][0] > 2.5:
            return np.ones([1, 2])
        else:
            return 0.2 * x + 0.5
