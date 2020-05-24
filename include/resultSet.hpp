#ifndef HW2_RESULTSET_HPP
#define HW2_RESULTSET_HPP


#include <iostream>
#include <vector>

#define ElemType float

class DistIndex
{
public:
    ElemType distance;
    int index;
    DistIndex(ElemType dist, int idx) : distance(dist), index(idx) {};

    bool operator < (const DistIndex& other)
    {
        return this->distance < other.distance;
    }
};

std::ostream& operator << (std::ostream& os, const DistIndex& DI);


class KNNResultSet
{
private:
    int capacity;
    ElemType worstDist = 1e10;

public:
    explicit KNNResultSet(int capa)
    {
        capacity = capa;
        distIndexList.reserve(capacity);

        for (int i = 0; i < capacity; i++)
            distIndexList.emplace_back(DistIndex(worstDist, 0));
    }

    int count = 0;
    int comparisionCount = 0;
    std::vector<DistIndex> distIndexList;

    int size()
    {
        return capacity;
    }

    ElemType getWorstDist()
    {
        return worstDist;
    }

    void list()
    {
        std::cout << "Distance-Index list: " << std::endl;
        for (const auto& item : distIndexList)
            std::cout << item << std::endl;
    }

    void addPoint(ElemType dist, int index)
    {
        comparisionCount++;

        if (dist > worstDist)
            return;

        if (count < capacity)
            count++;

        int currentPosition = count - 1;

        while (currentPosition > 0)
        {
            if (distIndexList[currentPosition - 1].distance > dist)
            {
                distIndexList[currentPosition] = distIndexList[currentPosition - 1];
                currentPosition--;
            }
            else
                break;
        }

        distIndexList[currentPosition].distance = dist;
        distIndexList[currentPosition].index = index;
        worstDist = distIndexList[capacity - 1].distance;
    }

};


class RadiusNNResultSet
{
private:
    ElemType worstDist;
    ElemType radius;

public:
    explicit RadiusNNResultSet(ElemType r)
    {
        radius = r;
        worstDist = r;
    }

    int count = 0;
    int comparisionCount = 0;
    std::vector<DistIndex> distIndexList;

    int size()
    {
        return count;
    }

    ElemType getWorstDist()
    {
        return worstDist;
    }

    void list()
    {
        std::cout << "Distance-Index list: " << std::endl;
        for (const auto& item : distIndexList)
            std::cout << item << std::endl;
    }

    void addPoint(ElemType dist, int index)
    {
        comparisionCount++;
        if (dist <= worstDist)
        {
            distIndexList.emplace_back(DistIndex(dist, index));
            count++;
        }
        else
            return;
    }

};


#endif //HW2_RESULTSET_HPP



