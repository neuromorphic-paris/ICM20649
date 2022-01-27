/**
 * @file
 * @brief Utility functions.
 */

#ifndef ICM20649_UTILS_HPP
#define ICM20649_UTILS_HPP

#include <termios.h>

namespace ICM20649
{
/**
 * @brief Reads a character from the stdin stream without blocking.
 *
 * This function reads a character from the stdin stream without blocking.
 *
 * @return Character read.
 */
int
get_key()
{
  // set the terminal to raw mode
  struct termios orig_term_attr;
  tcgetattr(fileno(stdin), &orig_term_attr);
  struct termios new_term_attr;
  memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
  new_term_attr.c_lflag &= ~(ECHO | ICANON);
  new_term_attr.c_cc[VTIME] = 0;
  new_term_attr.c_cc[VMIN] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

  // read a character from the stdin stream without blocking
  // returns EOF (-1) if no character is available
  const int character = fgetc(stdin);

  // restore the original terminal attributes
  tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

  return character;
}
}  // namespace ICM20649

#endif
