#include <deque>
#include <iostream>
#define MAX_VAL 4

int main()
{
  std::deque<int> Q;

  /* insert the even numbers 0,2,4,...2*MAX_VAL into Q */
  for (int i = 0; i <= MAX_VAL; ++i)
  {
    Q.push_back(2 * i);
  }

  int *my_ptr = &Q[2];
  std::cout << "ptr apunta a=" << *my_ptr << std::endl;

  int a = Q.front();
  std::cout << "ptr apunta a=" << *my_ptr << std::endl;
  Q.pop_front();
  std::cout << "ptr apunta a=" << *my_ptr << std::endl;

  std::cout << "===============" << std::endl;
  Q.pop_front();
  std::cout << "ptr apunta a=" << *my_ptr << std::endl;

  std::cout << "===============" << std::endl;
  Q.pop_front();
  std::cout << "ptr apunta a=" << *my_ptr << std::endl;

  std::cout << "===============" << std::endl;
  Q.pop_front();
  std::cout << "ptr apunta a=" << *my_ptr << std::endl;

  std::cout << "===============" << std::endl;
  Q.pop_front();
  std::cout << "ptr apunta a=" << *my_ptr << std::endl;

  std::cout << "Q has " << Q.size() << " elements" << std::endl;

  std::cout << "===============" << std::endl;
  Q.pop_front();
  std::cout << "ptr apunta a=" << *my_ptr << std::endl;

  return 0;
}
