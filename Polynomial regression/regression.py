# Import libraries
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression

def sci_notation(number, sig_fig):
    ret_string = "{0:.{1:d}e}".format(number, sig_fig)
    a, b = ret_string.split("e")
    if float(a) >= 0:
        a = "+" + a
    # remove leading "+" and strip leading zeros
    b = int(b)
    if b == 0:
        return a
    else:
        return a + "*10^" + str(b)

# Get dataset
data = pd.read_csv('sensor.csv')

# Splitting variables
X = data.iloc[:, 1:2].values  # independent
y = data.iloc[:, 0].values  # dependent

# Train polynomial regression model on the whole dataset
pr = PolynomialFeatures(degree = 2)
X_poly = pr.fit_transform(X)
lr = LinearRegression()
lr.fit(X_poly, y)

# Predict results
y_pred = lr.predict(X_poly)  # Polynomial Regression
coefs = lr.coef_
function = str(round(lr.intercept_,4)) + " "
for i in range(1, len(coefs)):
    function += sci_notation(coefs[i], sig_fig=4) + "*x^" + str(i) + " "

print(function)

# Visualize real data with polynomial regression
X_grid = np.arange(min(X), max(X), 0.1)
X_grid = X_grid.reshape((len(X_grid), 1))
plt.scatter(X, y, color = 'lightcoral')
plt.plot(X, y_pred, color = 'firebrick')
plt.show()