import os
import numpy as np

from flightgym import TestEnv_v0
from flightgym import QuadrotorEnv_v1

def main():
  test_env = TestEnv_v0()
  obs = np.zeros(shape=(3, 3), dtype=np.float32)
  test_env.reset(obs)
  print(obs)

  quad_env1 = QuadrotorEnv_v1()
  obs = np.zeros(shape=(100, 9), dtype=np.float32)
  quad_env1.reset(obs)
  print(obs)

if __name__ == "__main__":
    main()
