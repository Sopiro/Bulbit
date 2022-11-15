#include "raytracer/bvh.h"
#include "raytracer/hittable.h"

BVH::BVH()
{
    nodeID = 0;
    root = nullNode;
    nodeCapacity = 32;
    nodeCount = 0;
    nodes = (Node*)malloc(nodeCapacity * sizeof(Node));
    memset(nodes, 0, nodeCapacity * sizeof(Node));

    // Build a linked list for the free list.
    for (int32 i = 0; i < nodeCapacity - 1; ++i)
    {
        nodes[i].next = i + 1;
    }
    nodes[nodeCapacity - 1].next = nullNode;
    freeList = 0;
}

BVH::~BVH()
{
    free(nodes);
    root = nullNode;
    nodeCount = 0;
}

int32 BVH::AllocateNode()
{
    if (freeList == nullNode)
    {
        assert(nodeCount == nodeCapacity);

        // Grow the node pool
        Node* oldNodes = nodes;
        nodeCapacity *= 2;
        nodes = (Node*)malloc(nodeCapacity * sizeof(Node));
        memcpy(nodes, oldNodes, nodeCount * sizeof(Node));
        memset(nodes + nodeCount, 0, nodeCount * sizeof(Node));
        free(oldNodes);

        // Build a linked list for the free list.
        for (int32 i = nodeCount; i < nodeCapacity - 1; ++i)
        {
            nodes[i].next = i + 1;
        }
        nodes[nodeCapacity - 1].next = nullNode;
        freeList = nodeCount;
    }

    int32 node = freeList;
    freeList = nodes[node].next;
    nodes[node].id = ++nodeID;
    nodes[node].parent = nullNode;
    nodes[node].child1 = nullNode;
    nodes[node].child2 = nullNode;
    ++nodeCount;

    return node;
}

void BVH::FreeNode(int32 node)
{
    assert(0 <= node && node <= nodeCapacity);
    assert(0 < nodeCount);

    nodes[node].id = 0;
    nodes[node].next = freeList;
    freeList = node;
    --nodeCount;
}

int32 BVH::Add(Hittable* object, const AABB& aabb)
{
    int32 newNode = AllocateNode();

    nodes[newNode].aabb = aabb;
    nodes[newNode].isLeaf = true;
    nodes[newNode].body = object;
    object->node = newNode;

    return newNode;
}

int32 BVH::Insert(Hittable* body, const AABB& aabb)
{
    int32 newNode = AllocateNode();

    nodes[newNode].aabb = aabb;
    nodes[newNode].isLeaf = true;
    nodes[newNode].body = body;
    nodes[newNode].parent = nullNode;
    body->node = newNode;

    if (root == nullNode)
    {
        root = newNode;
        return newNode;
    }

    // Find the best sibling for the new leaf

#if 1
    int32 bestSibling = root;
    double bestCost = SAH(Union(nodes[root].aabb, aabb));

    GrowableArray<std::pair<int32, double>, 256> stack;
    stack.Emplace(root, 0.0f);

    while (stack.Count() != 0)
    {
        int32 current = stack.Back().first;
        double inheritedCost = stack.Back().second;
        stack.Pop();

        AABB combined = Union(nodes[current].aabb, aabb);
        double directCost = SAH(combined);

        double cost = directCost + inheritedCost;
        if (cost < bestCost)
        {
            bestCost = cost;
            bestSibling = current;
        }

        inheritedCost += directCost - SAH(nodes[current].aabb);

        double lowerBoundCost = SAH(aabb) + inheritedCost;
        if (lowerBoundCost < bestCost)
        {
            if (!nodes[current].isLeaf)
            {
                stack.Emplace(nodes[current].child1, inheritedCost);
                stack.Emplace(nodes[current].child2, inheritedCost);
            }
        }
    }
#else
    // O(log n)
    // This method is faster when inserting a new node, but builds a slightly bad quality tree.
    int32 bestSibling = root;
    while (nodes[bestSibling].isLeaf == false)
    {
        int32 child1 = nodes[bestSibling].child1;
        int32 child2 = nodes[bestSibling].child2;

        double area = SAH(nodes[bestSibling].aabb);
        AABB combined = Union(nodes[bestSibling].aabb, aabb);
        double combinedArea = SAH(combined);

        double cost = combinedArea;
        double inheritanceCost = combinedArea - area;

        double cost1;
        if (nodes[child1].isLeaf)
        {
            cost1 = SAH(Union(nodes[child1].aabb, aabb)) + inheritanceCost;
        }
        else
        {
            double newArea = SAH(Union(nodes[child1].aabb, aabb));
            double oldArea = SAH(nodes[child1].aabb);
            cost1 = (newArea - oldArea) + inheritanceCost; // Lower bound cost required when descending down to child1
        }

        double cost2;
        if (nodes[child2].isLeaf)
        {
            cost2 = SAH(Union(nodes[child2].aabb, aabb)) + inheritanceCost;
        }
        else
        {
            double newArea = SAH(Union(nodes[child2].aabb, aabb));
            double oldArea = SAH(nodes[child2].aabb);
            cost2 = (newArea - oldArea) + inheritanceCost; // Lower bound cost required when descending down to child2
        }

        if (cost < cost1 && cost < cost2)
        {
            break;
        }

        if (cost1 < cost2)
        {
            bestSibling = child1;
        }
        else
        {
            bestSibling = child2;
        }
    }
#endif

    // Create a new parent
    int32 oldParent = nodes[bestSibling].parent;
    int32 newParent = AllocateNode();
    nodes[newParent].aabb = Union(aabb, nodes[bestSibling].aabb);
    nodes[newParent].isLeaf = false;
    nodes[newParent].body = nullptr;
    nodes[newParent].parent = oldParent;

    if (oldParent != nullNode)
    {
        if (nodes[oldParent].child1 == bestSibling)
        {
            nodes[oldParent].child1 = newParent;
        }
        else
        {
            nodes[oldParent].child2 = newParent;
        }

        nodes[newParent].child1 = bestSibling;
        nodes[newParent].child2 = newNode;
        nodes[bestSibling].parent = newParent;
        nodes[newNode].parent = newParent;
    }
    else
    {
        nodes[newParent].child1 = bestSibling;
        nodes[newParent].child2 = newNode;
        nodes[bestSibling].parent = newParent;
        nodes[newNode].parent = newParent;
        root = newParent;
    }

    // Walk back up the tree refitting ancestors' AABB and applying rotations
    int32 ancestor = nodes[newNode].parent;
    while (ancestor != nullNode)
    {
        int32 child1 = nodes[ancestor].child1;
        int32 child2 = nodes[ancestor].child2;

        nodes[ancestor].aabb = Union(nodes[child1].aabb, nodes[child2].aabb);

        Rotate(ancestor);

        ancestor = nodes[ancestor].parent;
    }

    return newNode;
}

void BVH::Remove(Hittable* body)
{
    if (body->node == nullNode)
    {
        return;
    }

    int32 node = body->node;
    int32 parent = nodes[node].parent;
    body->node = nullNode;

    if (parent != nullNode) // node is not root
    {
        int32 sibling = nodes[parent].child1 == node ? nodes[parent].child2 : nodes[parent].child1;

        if (nodes[parent].parent != nullNode) // sibling has grandparent
        {
            nodes[sibling].parent = nodes[parent].parent;

            int32 grandParent = nodes[parent].parent;
            if (nodes[grandParent].child1 == parent)
            {
                nodes[grandParent].child1 = sibling;
            }
            else
            {
                nodes[grandParent].child2 = sibling;
            }
        }
        else // sibling has no grandparent
        {
            root = sibling;

            nodes[sibling].parent = nullNode;
        }

        FreeNode(node);
        FreeNode(parent);

        int32 ancestor = nodes[sibling].parent;
        while (ancestor != nullNode)
        {
            int32 child1 = nodes[ancestor].child1;
            int32 child2 = nodes[ancestor].child2;

            nodes[ancestor].aabb = Union(nodes[child1].aabb, nodes[child2].aabb);

            ancestor = nodes[ancestor].parent;
        }
    }
    else // node is root
    {
        if (root == node)
        {
            root = nullNode;
            FreeNode(node);
        }
    }
}

void BVH::Rotate(int32 node)
{
    if (nodes[node].isLeaf)
    {
        return;
    }

    if (nodes[node].parent == nullNode)
    {
        return;
    }

    int32 parent = nodes[node].parent;

    int32 sibling;
    if (nodes[parent].child1 == node)
    {
        sibling = nodes[parent].child2;
    }
    else
    {
        sibling = nodes[parent].child1;
    }

    uint32 count = 2;
    double costDiffs[4];
    double nodeArea = SAH(nodes[node].aabb);

    costDiffs[0] = SAH(Union(nodes[sibling].aabb, nodes[nodes[node].child1].aabb)) - nodeArea;
    costDiffs[1] = SAH(Union(nodes[sibling].aabb, nodes[nodes[node].child2].aabb)) - nodeArea;

    if (nodes[sibling].isLeaf == false)
    {
        double siblingArea = SAH(nodes[sibling].aabb);
        costDiffs[2] = SAH(Union(nodes[node].aabb, nodes[nodes[sibling].child1].aabb)) - siblingArea;
        costDiffs[3] = SAH(Union(nodes[node].aabb, nodes[nodes[sibling].child2].aabb)) - siblingArea;

        count += 2;
    }

    uint32 bestDiffIndex = 0;
    for (uint32 i = 1; i < count; ++i)
    {
        if (costDiffs[i] < costDiffs[bestDiffIndex])
        {
            bestDiffIndex = i;
        }
    }

    // Rotate only if it reduce the suface area
    if (costDiffs[bestDiffIndex] < 0.0f)
    {
        // printf("Tree rotation occurred: %d\n", bestDiffIndex);

        switch (bestDiffIndex)
        {
        case 0: // Swap(sibling, node->child2);
            if (nodes[parent].child1 == sibling)
                nodes[parent].child1 = nodes[node].child2;
            else
                nodes[parent].child2 = nodes[node].child2;

            nodes[nodes[node].child2].parent = parent;

            nodes[node].child2 = sibling;
            nodes[sibling].parent = node;

            nodes[node].aabb = Union(nodes[sibling].aabb, nodes[nodes[node].child1].aabb);
            break;
        case 1: // Swap(sibling, node->child1);
            if (nodes[parent].child1 == sibling)
                nodes[parent].child1 = nodes[node].child1;
            else
                nodes[parent].child2 = nodes[node].child1;

            nodes[nodes[node].child1].parent = parent;

            nodes[node].child1 = sibling;
            nodes[sibling].parent = node;

            nodes[node].aabb = Union(nodes[sibling].aabb, nodes[nodes[node].child2].aabb);
            break;
        case 2: // Swap(node, sibling->child2);
            if (nodes[parent].child1 == node)
                nodes[parent].child1 = nodes[sibling].child2;
            else
                nodes[parent].child2 = nodes[sibling].child2;

            nodes[nodes[sibling].child2].parent = parent;

            nodes[sibling].child2 = node;
            nodes[node].parent = sibling;

            nodes[sibling].aabb = Union(nodes[node].aabb, nodes[nodes[sibling].child2].aabb);
            break;
        case 3: // Swap(node, sibling->child1);
            if (nodes[parent].child1 == node)
                nodes[parent].child1 = nodes[sibling].child1;
            else
                nodes[parent].child2 = nodes[sibling].child1;

            nodes[nodes[sibling].child1].parent = parent;

            nodes[sibling].child1 = node;
            nodes[node].parent = sibling;

            nodes[sibling].aabb = Union(nodes[node].aabb, nodes[nodes[sibling].child1].aabb);
            break;
        }
    }
}

void BVH::Reset()
{
    nodeID = 0;
    root = nullNode;
    nodeCount = 0;
    memset(nodes, 0, nodeCapacity * sizeof(Node));

    // Build a linked list for the free list.
    for (int32 i = 0; i < nodeCapacity - 1; ++i)
    {
        nodes[i].next = i + 1;
    }
    nodes[nodeCapacity - 1].next = nullNode;
    freeList = 0;
}

void BVH::ReBuild()
{
    int32* leaves = (int32*)malloc(nodeCount * sizeof(Node));
    int32 count = 0;

    // Build an array of leaf node
    for (int32 i = 0; i < nodeCapacity; ++i)
    {
        // Already in the free list
        if (nodes[i].id == 0)
        {
            continue;
        }

        // Clean the leaf
        if (nodes[i].isLeaf)
        {
            nodes[i].parent = nullNode;

            leaves[count++] = i;
        }
        else
        {
            // Free the internal node
            FreeNode(i);
        }
    }

    while (count > 1)
    {
        double minCost = DBL_MAX;
        int32 minI = -1;
        int32 minJ = -1;

        // Find the best aabb pair
        for (int32 i = 0; i < count; ++i)
        {
            AABB aabbI = nodes[leaves[i]].aabb;

            for (int32 j = i + 1; j < count; ++j)
            {
                AABB aabbJ = nodes[leaves[j]].aabb;

                AABB combined = Union(aabbI, aabbJ);
                double cost = SAH(combined);

                if (cost < minCost)
                {
                    minCost = cost;
                    minI = i;
                    minJ = j;
                }
            }
        }

        int32 index1 = leaves[minI];
        int32 index2 = leaves[minJ];
        Node* child1 = nodes + index1;
        Node* child2 = nodes + index2;

        // Create a parent(internal) node
        int32 parentIndex = AllocateNode();
        Node* parent = nodes + parentIndex;

        parent->child1 = index1;
        parent->child2 = index2;
        parent->aabb = Union(child1->aabb, child2->aabb);
        parent->parent = nullNode;

        child1->parent = parentIndex;
        child2->parent = parentIndex;

        leaves[minI] = parentIndex;

        leaves[minJ] = leaves[count - 1];
        --count;
    }

    root = leaves[0];
    free(leaves);
}

double BVH::ComputeCost() const
{
    double cost = 0.0;

    if (root == nullNode)
    {
        return cost;
    }

    GrowableArray<int32, 256> stack;
    stack.Emplace(root);

    while (stack.Count() != 0)
    {
        int32 current = stack.Pop();

        if (!nodes[current].isLeaf)
        {
            stack.Emplace(nodes[current].child1);
            stack.Emplace(nodes[current].child2);
        }

        const Node* node = nodes + current;

        cost += SAH(node->aabb);
    }

    return cost;
}

void BVH::RayCast(const Ray& r,
                  double t_min,
                  double t_max,
                  const std::function<double(const Ray&, double, double, Hittable*)>& callback) const
{
    Vec3 p1 = r.At(t_min);
    Vec3 p2 = r.At(t_max);
    double t = t_max;

    AABB rayAABB;
    rayAABB.min = Min(p1, p2);
    rayAABB.max = Max(p1, p2);

    GrowableArray<int32, 256> stack;
    stack.Emplace(root);

    while (stack.Count() > 0)
    {
        int32 nodeID = stack.Pop();
        if (nodeID == nullNode)
        {
            continue;
        }

        const Node* node = nodes + nodeID;
        if (TestOverlapAABB(node->aabb, rayAABB) == false)
        {
            continue;
        }

        if (node->aabb.Hit(r, t_min, t) == false)
        {
            continue;
        }

        if (node->isLeaf)
        {
            double value = callback(r, t_min, t, node->body);
            if (value <= t_min)
            {
                return;
            }
            else
            {
                // Update ray AABB
                t = value;
                Vec3 newEnd = r.At(t);

                rayAABB.min = Min(p1, newEnd);
                rayAABB.max = Max(p1, newEnd);
            }
        }
        else
        {
            stack.Emplace(node->child1);
            stack.Emplace(node->child2);
        }
    }
}