#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

using namespace std;
int main()
{
  char batata;
  string arry[5];
  vector <int> array;
  array.reserve(5);
  int i;
  ifstream infile("./file.txt"); // for example
  string line = "";

  while (getline(infile, line))
  {
    stringstream strstr(line);
    string word = "";
    while (getline(strstr,word, ':'))
    {
      stringstream str(word);
      for(i = 0; i < 5; i++)
      {
	getline(str, arry[i], ',');
	array.push_back(atoi(arry[i].c_str()));
	cout<<array[i]<<".";
      }
      array.clear(); //dar output antes de dar clear
      //cout <<"\n"<< word << '\n';
      cin>>batata;     //wait();
    }
  }
}