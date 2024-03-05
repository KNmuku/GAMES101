#include <algorithm>
#include <cassert>
#include <ios>
#include "BVH.hpp"

using namespace std;


BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    /*
    if (objects.size() == 0) {
        return nullptr;
    }*/

    BVHBuildNode* node = new BVHBuildNode();

    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        if (splitMethod == SplitMethod::NAIVE) {
            Bounds3 centroidBounds;
            for (int i = 0; i < objects.size(); ++i)
                centroidBounds =
                    Union(centroidBounds, objects[i]->getBounds().Centroid());
            int dim = centroidBounds.maxExtent();
            switch (dim) {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                           f2->getBounds().Centroid().x;
                });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                           f2->getBounds().Centroid().y;
                });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                           f2->getBounds().Centroid().z;
                });
                break;
            }

            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        } else if (splitMethod == SplitMethod::SAH) {

            /* Surface Area Heuristic */
            // Compute bounds of all primitives in BVH node
            Bounds3 bounds;
            for (int i = 0; i < objects.size(); ++i) {
                bounds = Union(bounds, objects[i]->getBounds());
            }
            Vector3f most_left_pos = bounds.pMin;
            Vector3f most_right_pos = bounds.pMax;

            auto compute_bucket = [](float left, float right, float position, int n_buckets) {
                return (int) clamp(0, n_buckets-0.001, (position - left) * n_buckets / (right - left));
            };

            // record important information for minimum cost splitting
            int split_plane;
            int chosen_dim;
            float min_cost = kInfinity;
            vector<vector<Object *>> objs_in_buckets(BVHAccel::num_of_buckets);
        
            // for each axis x, y, z;
            for (int dim = 0; dim < 3; ++dim) {
                // initialize buckets with all empty bboxes;
                vector<Bounds3> buckets(BVHAccel::num_of_buckets);
                vector<int> obj_in_buckets_count(BVHAccel::num_of_buckets);
                // for each objects p in this node;
                for (const auto obj_ptr : objects) {
                    Bounds3 obj_box = obj_ptr->getBounds();
                    float left, right;
                    float this_obj_pos;
                    if (dim == 0) {
                        left = most_left_pos.x;
                        right = most_right_pos.x;
                        this_obj_pos = obj_box.Centroid().x;
                    } else if (dim == 1) {
                        left = most_left_pos.y;
                        right = most_right_pos.y;
                        this_obj_pos = obj_box.Centroid().y;
                    } else {
                        left = most_left_pos.z;
                        right = most_right_pos.z;
                        this_obj_pos = obj_box.Centroid().z;
                    }
                    int bucket_index = compute_bucket(left, right, this_obj_pos, BVHAccel::num_of_buckets);
                    buckets.at(bucket_index) = Union(buckets.at(bucket_index), obj_box);
                    ++obj_in_buckets_count.at(bucket_index);
                }
                /*
                 * * * * * * * * * * * * * * * * * *
                 * plane 0     1     2     3     4 *
                 * part  | -0- | -1- | -2- | -3- | *
                 * * * * * * * * * * * * * * * * * *
                 */
                // for each of the B-1 possible partitioning planes evaluate SAH and found the lowest cost
                for (int plane = 1; plane < BVHAccel::num_of_buckets; ++plane) {
                    Bounds3 left_box, right_box;
                    int left_num = 0, right_num = 0;
                    for (int part = 0; part < plane; ++part) {
                        left_box = Union(left_box, buckets.at(part));
                        left_num += obj_in_buckets_count.at(part);
                    }
                    for (int part = plane; part < BVHAccel::num_of_buckets; ++part) {
                        right_box = Union(right_box, buckets.at(part));
                        right_num += obj_in_buckets_count.at(part);
                    }
                    float cost = left_box.SurfaceArea() * left_num + right_box.SurfaceArea() * right_num /* * bounds.SurfaceArea()*/; 
                
                    if (cost < min_cost) {
                        chosen_dim = dim;
                        min_cost = cost;
                        split_plane = plane; 
                    }
                }
            }

            vector<Object *> left_node_objs, right_node_objs;
            Bounds3 obj_box;
            float left, right;
            float this_obj_pos;

            for (const auto obj_ptr : objects) {
                obj_box = obj_ptr->getBounds();
                if (chosen_dim == 0) {
                    left = most_left_pos.x;
                    right = most_right_pos.x;
                    this_obj_pos = obj_box.Centroid().x;
                } else if (chosen_dim == 1) {
                    left = most_left_pos.y;
                    right = most_right_pos.y;
                    this_obj_pos = obj_box.Centroid().y;
                } else {
                    left = most_left_pos.z;
                    right = most_right_pos.z;
                    this_obj_pos = obj_box.Centroid().z;
                }
                int bucket_index = compute_bucket(left, right, this_obj_pos, BVHAccel::num_of_buckets);
                if (bucket_index < split_plane) {
                    left_node_objs.push_back(obj_ptr);
                } else {
                    right_node_objs.push_back(obj_ptr);
                }
            }
            if (left_node_objs.size() + right_node_objs.size() != objects.size()) {
                throw runtime_error("splitting error");
            }
            node->bounds = bounds;
            node->left = recursiveBuild(left_node_objs);
            node->right = recursiveBuild(right_node_objs);
        }
    }
    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Vector3f vec_invert = {1.f/ray.direction.x, 1.f/ray.direction.y, 1.f/ray.direction.z};
    array<int, 3> is_positive = {int(ray.direction.x > 0),
                                 int(ray.direction.y > 0),
                                 int(ray.direction.z > 0)};
    // oh nononono!!! It should not be (int) ray.direction.x > 0
    // and GDB is yyds!!!
    
    if (!node || !node->bounds.IntersectP(ray, vec_invert, is_positive)) {
        return Intersection(); 
    }
    if (!node->left && !node->right) {
        return node->object->getIntersection(ray);
    }
    Intersection left_inter = getIntersection(node->left, ray);
    Intersection right_inter = getIntersection(node->right, ray); 
    return left_inter.distance < right_inter.distance ? left_inter : right_inter;
}
