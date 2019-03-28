"""
Implements fast ica for two components
"""
import numpy as np
import sounddevice as sd
from scipy.io import wavfile
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize


def main():
    """
    Main program
    """

    ## Speech data ===========================================================
    fs, data1 = wavfile.read('./ertain-20160302/wav/cc-01.wav')
    fs, data2 = wavfile.read('./ertain-20160302/wav/cc-02.wav')

    data1 = np.trim_zeros(data1)
    data2 = np.trim_zeros(data2)

    data1 = data1 / 65536
    data2 = data2 / 65536

    length = min(len(data1), len(data2))
    S = np.mat([data1[:length], data2[:length]])
    ## =======================================================================


    ## SINE function ==========================================================
    # np.random.seed(0)
    # sample_rate = 16000
    # numseconds = .1
    # freq1 = 440
    # freq2 = 261.63

    # timebase = np.linspace(0, numseconds, numseconds * sample_rate)

    # s_1 = np.sin(2 * np.pi * freq1 * timebase)  # signal 1 : sinusoidal signal
    # s_2 = np.sin(2 * np.pi * freq2 * timebase)  # signal 1 : sinusoidal signal

    # s_1 = s_1 + np.random.normal(size=s_1.size)
    # s_2 = s_2 + np.random.normal(size=s_2.size)
    ## SINE function ==========================================================

    # S = np.mat([s_1, s_2])
    # S += 0.2 * np.random.normal(size=S.shape)  # Add noise
    #===========================================================================

    # Generate random mixing matrix
    # A = np.random.rand(2, 2)

    # Or predefined mixing matrix
    A = np.mat([[1, 2], [1, 1]])

    X = np.dot(A, S)  # Generate observations

    # Perform FastICA
    est = fast_ica(X)

    print("Playing Original")
    sd.play(np.array(S[0]).squeeze(), fs)
    sd.wait()
    sd.play(np.array(S[1]).squeeze(), fs)
    sd.wait()

    print("Playing Mixed")
    sd.play(np.array(X[0]).squeeze(), fs)
    sd.wait()
    sd.play(np.array(X[1]).squeeze(), fs)
    sd.wait()

    print("FastICA output")
    sd.play(np.array(est[0]).squeeze() / 10, fs)
    sd.wait()
    sd.play(np.array(est[1]).squeeze() / 10, fs)
    sd.wait()


    # Plot output ===================================================
    # plt.subplot(3, 1, 1)
    # plt.title('Fast ICA output')
    # plt.plot(est.T)
    # plt.subplot(3, 1, 2)
    # plt.title('Original source signal')
    # plt.plot(S.T)
    # plt.subplot(3, 1, 3)
    # plt.title('Mixed signal')
    # plt.plot(X.T)
    # plt.show()
    # ===============================================================


def eigen2by2(mat):
    """
    Compute eigen values and vectors of two by two matrix
    mat = [[a, b],
           [c, d]]
    """
    a = mat[0,0]
    b = mat[0,1]
    c = mat[1,0]
    d = mat[1,1]

    tr = a + d
    det = a * d - b * c

    i = np.mat([[1,0], [0,1]])

    eigval1 = (tr + np.sqrt(tr ** 2 - 4 * det)) / 2
    eigval2 = (tr - np.sqrt(tr ** 2 - 4 * det)) / 2

    ev1 = mat - eigval1 * i
    ev2 = mat - eigval2 * i

    eigvec = np.mat([[ev1[0,0], ev2[0, 0]], [ev1[1,0], ev2[1, 0]]])
    eigval = np.mat([[eigval2, 0], [0, eigval1]])
    eigvec = eigvec / np.linalg.norm(eigvec, axis=0)

    return (eigval, eigvec)



def fast_ica(input_mat, max_num_iterations=1000, epsilon=0.0001):
    """Implement fast ica algorithm

    Args:
        X (TODO): (num_component, num_sample)

    Returns: TODO

    """

    (num_comp, num_sample) = np.shape(input_mat)

    #==== Prewhitening data ====#
    # Center matrix
    mean = np.mean(input_mat, axis=1)
    center_mat = np.mat(input_mat - mean)

    # Find covariance matrix cov_mat = np.cov(center_mat.T)
    cov_mat = center_mat * center_mat.T / (num_sample - 1)

    # Find eigenvalues and eigenvector of the covariance matrix
    (eig_val, eig_vec) = eigen2by2(cov_mat)
    # (eig_val, eig_vec) = np.linalg.eig(cov_mat)
    # eig_val = np.mat(np.diagflat(eig_val))
    # eig_vec = np.mat(eig_vec)

    # Whiten input matrix
    whitening_mat = np.linalg.inv(np.sqrt(eig_val)) * eig_vec.T
    dewhitening_mat = eig_vec * np.sqrt(eig_val)
    white_mat = whitening_mat * center_mat

    # two basis vectors are orthogonal. only need to run once
    # the other one is rotate 90 deg

    # initialize a random vector
    weight = np.mat(np.random.normal(size=(num_comp, 1)))

    # normalize the weight
    weight = weight / np.linalg.norm(weight)

    # keep a history matrix
    weight_old = np.mat(np.zeros_like(weight))

    iteration = 0
    while iteration < max_num_iterations:

        # Test for convergence
        if np.linalg.norm(weight - weight_old) < epsilon \
                or np.linalg.norm(weight + weight_old) < epsilon:
            print("converged")
            break

        # update weight
        weight_old = weight
        weight = (white_mat * np.power(white_mat.T * weight, 3)) \
            / num_sample - 3 * weight

        # normalize the weight
        weight = weight / np.linalg.norm(weight)

    basis_set = np.mat(np.zeros((num_comp, num_comp)))
    basis_set[:, 0] = weight
    basis_set[:, 1] = np.matrix([[0,-1], [1,0]]) * weight 

    dewhiten_vec = dewhitening_mat * basis_set
    ica_fltr = basis_set.T * whitening_mat

    return  ica_fltr * np.mat(input_mat)


if __name__ == "__main__":
    main()


