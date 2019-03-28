# FastICA for two components implemented in python

Just a even more simplified version of the fast ica implemented in python.
Since the second components is orthogonal to the first one once the input
is whitened, we don't have to iterate again.

The example separates speech signals. You can comment it out and uncomment
the sine generating part to see the algorithm run on a sine wave.
