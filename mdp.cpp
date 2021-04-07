#include <bits/stdc++.h>
#include <cstdlib>
#include <ctime>
#include <cmath>
using namespace std;
struct hash_pair {
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& p) const
    {
        auto hash1 = hash<T1>{}(p.first);
        auto hash2 = hash<T2>{}(p.second);
        return hash1 ^ hash2;
    }
};
void print_vals(vector<vector<float>> value){
    for(int i=0;i<value.size();i++){
        for(int j=0;j<value[0].size();j++){
            cout << setw (10);
            cout<<value[i][j]<<"\t";
        }
        cout<<"\n";
    }
    cout<<"\n\n";
}
int main(int argc, char **argv){
    float abs = 0.01;
    float gamma = 0.9;
    float noise = 0.1;
    vector<pair<int,int>> states;
    for(int i=0;i<3;i++){
        for(int j=0;j<4;j++){
            states.push_back(make_pair(i,j));
        }
    }
    vector<vector<int>> rewards (3,vector<int>(4,0));
    rewards[2][2] = -1;
    rewards[1][2] = -1;
    rewards[2][3] = 1;

    unordered_map<pair<int,int>,vector<string>,hash_pair> actions;
    actions[make_pair(0,0)] = {"D","R"};
    actions[make_pair(0,1)] = {"D","R","L"};
    actions[make_pair(0,2)] = {"D","L","R"};
    actions[make_pair(0,3)] = {"D","L"};
    actions[make_pair(1,0)] = {"D","U","R"};
    actions[make_pair(1,1)] = {"D","R","L","U"};
    actions[make_pair(1,3)] = {"D","L","U"};
    actions[make_pair(2,0)] = {"U","R"};
    actions[make_pair(2,1)] = {"U","L","R"};

    unordered_map<pair<int,int>,string,hash_pair> policy;
    int rI;
    for(auto i:actions){
        srand (static_cast <unsigned> (time(0)));
        rI = rand() % actions[i.first].size();
        policy[i.first] = actions[i.first][rI];
    }

    vector<vector<float>> value (3,vector<float>(4,0));
    value[2][2] = -1;
    value[1][2] = -1;
    value[2][3] = 1;

    int iteration = 0;
    float change,old_val,new_val,val;
    change = INT_MIN;
    string rand_str;
    pair<int,int> next_pair,random_pair;
    int iter = 0;
    while(1){
        change = 0;
        for(auto p:states){
            if(policy.find(p)!=policy.end()){
                old_val = value[p.first][p.second];
                new_val = FLT_MIN;
                for(auto a:actions[p]){
                    if (a == "U"){
                        next_pair = make_pair(p.first-1, p.second);
                    }
                    if (a == "D"){
                        next_pair = make_pair(p.first+1, p.second);
                    }
                    if (a == "L"){
                        next_pair = make_pair(p.first, p.second-1);
                    }
                    if (a == "R"){
                        next_pair = make_pair(p.first, p.second+1);
                    }

                    rI = rand() % actions[p].size();
                    rand_str = actions[p][rI];
                    if (rand_str == "U"){
                        random_pair = make_pair(p.first-1, p.second);
                    }
                    if (rand_str == "D"){
                        random_pair = make_pair(p.first+1, p.second);
                    }
                    if (rand_str == "L"){
                        random_pair = make_pair(p.first, p.second-1);
                    }
                    if (rand_str == "R"){
                        random_pair = make_pair(p.first, p.second+1);
                    }

                    val = rewards[p.first][p.second] + (gamma * ((1-noise)* value[next_pair.first][next_pair.second] + (noise * value[random_pair.first][random_pair.second]))); 
                    if(val>new_val){
                        new_val = val;
                        policy[p] = a;
                    }
                }
                value[p.first][p.second] = new_val;
                change = max(change, fabs(old_val - new_val));         
            }
        }
        if(change<abs){
            break;
        }   
        iter++;
        cout<<iter<<":"<<"\n\n";
        // print_vals(value);

    }
    cout<<"Final values after "<<iter<<" iterations :\n";
    print_vals(value);
    return 0;
}