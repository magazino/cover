#include "cover.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>

using namespace cover;

int
main(int argc, char** argv) {
  std::cout << "input with " << argc << std::endl;

  const auto cols = (argc - 1) / 2;
  cover::polygon outline(2, cols);

  for (int cc = 0, ii = 1; cc != outline.cols(); ++cc) {
    outline(0, cc) = std::atof(argv[ii++]);
    outline(1, cc) = std::atof(argv[ii++]);
  }

  std::cout << outline << std::endl;

  try {
    const auto res = split(outline, 1);
    std::cout << res.ring << std::endl;
    std::cout << "dense elements " << std::endl;
    for (const auto& dense : res.dense)
      std::cout << dense << std::endl;
  }
  catch (std::exception& _ex) {
    std::cerr << _ex.what() << std::endl;
  }
  return 0;
}
