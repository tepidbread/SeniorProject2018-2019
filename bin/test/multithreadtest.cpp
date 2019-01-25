#include <thread>
#include <mutex>
#include <math.h>
#include <iostream>
using namespace std;

std::mutex m_a;	//creating the mutex
std::mutex m_b;	//creating the mutex
std::mutex m_c;	//creating the mutex

void calc_a(int *a){
	while (*a<10000){
		m_a.lock();
			*a = *a + 1;
		m_a.unlock();
	}
}

void calc_b(int *b){
	while (*b<10000){
		m_b.lock();
			*b = *b + 1;
		m_b.unlock();
	}
}

void calc_c(int *c){
	while (*c<10000){
		m_c.lock();
			*c = *c + 1;
		m_c.unlock();
	}
}

main(){
	int a = 0,b = 0,c = 0;


	std::thread t1(calc_a,&a);
	std::thread t2(calc_b,&b);
	std::thread t3(calc_c,&c);

	t1.detach();
	t2.detach();
	t3.detach();

	while(1){

		cout << "a =\t" << a << endl;
		cout << "b =\t" << b << endl;
		cout << "c =\t" << c << endl;
		cout << endl;
	}

}
