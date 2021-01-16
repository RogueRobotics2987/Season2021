<<<<<<< HEAD
#include <hal/HAL.h>

#include "gtest/gtest.h"

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
=======
#include <hal/HAL.h>

#include "gtest/gtest.h"

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
>>>>>>> 119e8b76d8cbbb98afe2ba5b6ee254a4c9262844
