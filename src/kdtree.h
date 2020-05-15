/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_H_
#define KDTREE_H_
// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        Node** nodePtr = &root;
        auto itr = begin(point);
        while(*nodePtr != NULL)
        {           
            auto node = *nodePtr;
            auto dim = distance(begin(point),itr);
            auto dimval = begin(node->point);
            std::advance(dimval, dim);
            
            if(*itr < *dimval)
            {
                nodePtr = &(node->left);
            }
            else
            {
                nodePtr = &(node->right);
            }
            
            itr ++;
            if(itr == end(point) )
            {
              itr = begin(point);
            }
        }
        *nodePtr = new Node(point,id);
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float>& target, float distanceTol)
    {
        std::vector<int> ids;
        Node** nodePtr = &root;
        Node** nodePtr2 = NULL;
        auto itr = begin(target);
        std::vector<float> tolerence_min;
        std::vector<float> tolerence_max;
        
        std::transform(begin(target),end(target),back_inserter(tolerence_min),[=](float v){return v-distanceTol;});
        std::transform(begin(target),end(target),back_inserter(tolerence_max),[=](float v){return v+distanceTol;});
        
        std::stack<std::pair<Node *,int>> nodeStack;
        if (root != NULL)
        {
            nodeStack.push(std::make_pair(root,0));
        }
        while (nodeStack.empty() == false)
        {
            auto pair = nodeStack.top();
            Node* node = pair.first;
            int level = pair.second;
            int dim = level % target.size();
            //node within box?
            //if(std::equal (begin(target), end(target), begin(node->point), [=](float v,float t){return fabs(v-t)<=distanceTol;} ) )
            if ((fabs(target[0]-node->point[0]) <= distanceTol) &&
              (fabs(target[1]-node->point[1]) <= distanceTol) &&
              (fabs(target[2]-node->point[2]) <= distanceTol))
            {
                //No Mismatch, point is in the box
                //check distances
                //float distance = sqrt( std::inner_product (begin(target), end(target), begin(node->point),0.0, [=](float a,float b){return (a - b)*(a - b);},std::plus<float>()) );
                float distance = sqrt(
                  ((target[0]-node->point[0])*(target[0]-node->point[0]))+
                  ((target[1]-node->point[1])*(target[1]-node->point[1]))+
                  ((target[2]-node->point[2])*(target[2]-node->point[2]))
                                     ); 
                if(distanceTol >= distance)
                {
                    ids.push_back(node->id);
                }
            }
            
            nodeStack.pop();
            level ++;
            if ((node->left) && (tolerence_min[dim] < node->point[dim]))
                nodeStack.push(std::make_pair(node->left,level));
            if ((node->right) && (tolerence_max[dim] > node->point[dim]))
                nodeStack.push(std::make_pair(node->right,level));
        }
        
        return ids;
    }
};

#endif 