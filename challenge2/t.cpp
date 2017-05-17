
#include <iostream>
#include <string>

using namespace std;

int main(){
  ios::sync_with_stdio(false);
  string str;
  int i = 0;
  while(getline(cin, str)){
    cout << i++ << str << endl;
  }

  return 0;
}
