import numpy as np
import random
from math import sqrt, pi, exp

def gaussian_prob(obs, mu, sig):
    # Calculate Gaussian probability given
    # - observation
    # - mean
    # - standard deviation
    num = (obs - mu) ** 2
    denum = 2 * sig ** 2
    norm = 1 / sqrt(2 * pi * sig ** 2)
    return norm * exp(-num / denum)

# Gaussian Naive Bayes class
class GNB():
    # Initialize classification categories
    def __init__(self):
        self.classes = ['left', 'keep', 'right']
        self.prior = {}
        self.prior[self.classes[0]] = {}
        self.prior[self.classes[1]] = {}
        self.prior[self.classes[2]] = {}


    # Given a set of variables, preprocess them for feature engineering.
    def process_vars(self, vars):
        # The following implementation simply extracts the four raw values
        # given by the input data, i.e. s, d, s_dot, and d_dot.
        s, d, s_dot, d_dot = vars
        # if d >= 6 :
        #     d -= 8
        # elif d >= 2 :
        #     d -= 4
        # else :
        #     d = d
        return s, d, s_dot, d_dot

    # Train the GNB using a combination of X and Y, where
    # X denotes the observations (here we have four variables for each) and
    # Y denotes the corresponding labels ("left", "keep", "right").
    def train(self, X, Y):
        '''
        Collect the data and calculate mean and standard variation
        for each class. Record them for later use in prediction.
        '''
        # TODO: implement code.
        x_1 = {'left':[], 'keep':[], 'right':[]}
        x_2 = {'left':[], 'keep':[], 'right':[]}
        x_3 = {'left':[], 'keep':[], 'right':[]}
        x_4 = {'left':[], 'keep':[], 'right':[]}

        for x, y in zip(X, Y) :
            if y == 'left' :
                x_1['left'].append(self.process_vars(x)[0])
                x_2['left'].append(self.process_vars(x)[1])
                x_3['left'].append(self.process_vars(x)[2])
                x_4['left'].append(self.process_vars(x)[3])
            elif y == 'keep' :
                x_1['keep'].append(self.process_vars(x)[0])
                x_2['keep'].append(self.process_vars(x)[1])
                x_3['keep'].append(self.process_vars(x)[2])
                x_4['keep'].append(self.process_vars(x)[3])
            elif y == 'right':
                x_1['right'].append(self.process_vars(x)[0])
                x_2['right'].append(self.process_vars(x)[1])
                x_3['right'].append(self.process_vars(x)[2])
                x_4['right'].append(self.process_vars(x)[3])


        self.prior[self.classes[0]]['x1'] = [np.mean(x_1['left']), np.std(x_1['left'])]
        self.prior[self.classes[0]]['x2'] = [np.mean(x_2['left']), np.std(x_2['left'])]
        self.prior[self.classes[0]]['x3'] = [np.mean(x_3['left']), np.std(x_3['left'])]
        self.prior[self.classes[0]]['x4'] = [np.mean(x_4['left']), np.std(x_4['left'])]

        self.prior[self.classes[1]]['x1'] = [np.mean(x_1['keep']), np.std(x_1['keep'])]
        self.prior[self.classes[1]]['x2'] = [np.mean(x_2['keep']), np.std(x_2['keep'])]
        self.prior[self.classes[1]]['x3'] = [np.mean(x_3['keep']), np.std(x_3['keep'])]
        self.prior[self.classes[1]]['x4'] = [np.mean(x_4['keep']), np.std(x_4['keep'])]

        self.prior[self.classes[2]]['x1'] = [np.mean(x_1['right']), np.std(x_1['right'])]
        self.prior[self.classes[2]]['x2'] = [np.mean(x_2['right']), np.std(x_2['right'])]
        self.prior[self.classes[2]]['x3'] = [np.mean(x_3['right']), np.std(x_3['right'])]
        self.prior[self.classes[2]]['x4'] = [np.mean(x_4['right']), np.std(x_4['right'])]


    # Given an observation (s, s_dot, d, d_dot), predict which behaviour
    # the vehicle is going to take using GNB.
    def predict(self, observation):
        '''
        Calculate Gaussian probability for each variable based on the
        mean and standard deviation calculated in the training process.
        Multiply all the probabilities for variables, and then
        normalize them to get conditional probabilities.
        Return the label for the highest conditional probability.
        '''
        # TODO: implement code.

        post_pro = []

        for i in range(len(self.classes)) :

            x1 = gaussian_prob(observation[0], self.prior[self.classes[i]]['x1'][0], self.prior[self.classes[i]]['x1'][1])
            x2 = gaussian_prob(observation[1], self.prior[self.classes[i]]['x2'][0], self.prior[self.classes[i]]['x2'][1])
            x3 = gaussian_prob(observation[2], self.prior[self.classes[i]]['x3'][0], self.prior[self.classes[i]]['x3'][1])
            x4 = gaussian_prob(observation[3], self.prior[self.classes[i]]['x4'][0], self.prior[self.classes[i]]['x4'][1])

            post_pro.append(x1 * x2 * x3 * x4)

        k = np.argmax(post_pro)

        return self.classes[k]

