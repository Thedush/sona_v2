import numpy as np

value = 299
window = [0, 100, 200, 300, 400, 500, 600, 700, 800]

matrix_size = len(window) - 1
windows_matrix = np.zeros(matrix_size)

for i in range(matrix_size):
    if window[i] <= value < window[i+1]:
        windows_matrix[i] = 1
        windows_matrix[i+1] = 1

print("Windows matrix:")
print(windows_matrix)
