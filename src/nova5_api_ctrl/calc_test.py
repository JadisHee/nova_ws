import numpy as np
import tf_transformations

# a = [0,0,1.570796327]
# a_mat = tf_transformations.euler_matrix(a[0],a[1],a[2])

# b = [0,3.141592653,0]
# b_mat = tf_transformations.euler_matrix(b[0],b[1],b[2])

# c = [0.00198,0,0]
# c_mat = tf_transformations.euler_matrix(c[0],c[1],c[2])

# d = [0,0,-0.03365]
# d_mat = tf_transformations.euler_matrix(d[0],d[1],d[2])


# e = [0,0.06981317,0]
# e_mat = tf_transformations.euler_matrix(e[0],e[1],e[2])

T = np.array([
    [0.033644,0.999317,0.015288,0.640],
    [-0.999432,0.033671,-0.001465,0.274],
    [-0.001979,-0.01523,0.999882,0.757],
    [0,0,0,1]
])
np.set_printoptions(suppress=True, precision=4)
t = tf_transformations.euler_from_matrix(T)
print(t)