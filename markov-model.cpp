#include <bits/stdc++.h>
#include <cstdlib>
#include <ctime>
using namespace std;
bool comp(pair<string, int> a, pair<string, int> b)
{
    return a.second > b.second;
}
void readData(unordered_map<string, vector<string>> &umap, string file)
{
    fstream fin(file.c_str());
    if (!fin.is_open())
    {
        cout << "Unable to read the file" << endl;
        exit(0);
        return;
    }
    string first, second;
    fin >> first;
    transform(first.begin(), first.end(), first.begin(), ::tolower);
    while (fin >> second)
    {
        transform(second.begin(), second.end(), second.begin(), ::tolower);
        if (umap.find(first) != umap.end())
        {
            umap[first].push_back(second);
        }
        else
        {
            vector<string> vec;
            vec.push_back(second);
            umap[first] = vec;
        }
        first = second;
    }
    unordered_map<string, int> u2;
    for (auto p : umap)
    {
        u2.clear();
        for (auto j : umap[p.first])
        {
            u2[j] += 1;
        }
        vector<pair<string, int>> vp;
        vp.clear();
        for (auto i : u2)
        {
            vp.push_back(i);
        }
        u2.clear();
        sort(vp.begin(), vp.end(), comp);
        vector<string> vect;
        vect.clear();
        for (auto k : vp)
        {
            for (int i = 0; i < k.second; i++)
            {
                vect.push_back(k.first);
            }
        }
        umap[p.first] = vect;
    }
    return;
}
void printmap(unordered_map<string, vector<string>> &umap)
{
    for (auto i : umap)
    {
        cout << i.first << " : ";
        for (auto j : umap[i.first])
        {
            cout << j << ", ";
        }
        cout << endl;
    }
}
int main(int argc, char **argv)
{
    unordered_map<string, vector<string>> umap;
    stringstream ss;
    string file = "speech.txt";
    readData(umap, file);
    // printmap(umap);
    int j = 20;
    string start = "potter";
    string ans = start;
    float prob;
    srand (static_cast <unsigned> (time(0)));
    while (j > 0)
    {
        int randomIndex = rand() % umap[start].size();
        float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        if (r < 0.15)
        {
            randomIndex = 0;
        }
        string next = umap[start][randomIndex];
        ans += " ";
        ans += next;
        start = next;
        j--;
    }
    cout << ans << endl;
    return 0;
}