#include <stdio.h>
#include <vector>

#define NODE_NUM 10

class List {
	private:
		int nodeID;
		int nodeID_Parent;
		float F_score;
		float G_score;
		float H_score;

	public:
		// float root[NODE_NUM][NODE_NUM];
		// 생성자
		List(int loopback, int parent {
			this->nodeID = loopback; 
			this->nodeID_Parent = parent; 
			// this->root = rootNode;
			};

		// 스코어 계산
		void updateF_SCORE(){this->F_score = this->G_score +this-> H_score;};
		void updateG_SCORE(float G_value) {this->G_score = G_value;};
		void updateH_SCORE(int destination);

};

void List::updateH_SCORE(int destination) {
	this->H_score = nodeID * nodeID - destination * destination;
}

int main() {
	//노드 리스트
	float root[10][10];

	//리스트
	std::vector<List> O_List;
	std::vector<List> C_List;



	C_List.push_back(List(0, 0));
}