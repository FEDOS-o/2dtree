#include "primitives.h"

#include <fstream>
#include <map>

namespace {
std::vector<Point> read(const char * filename)
{
    std::ifstream in;
    in.open(filename);
    std::vector<Point> result;
    double x, y;
    while (in >> x) {
        in >> y;
        result.emplace_back(x, y);
    }
    return result;
}
} // namespace

namespace kdtree {
using iterator = PointSet::iterator;
PointSet::PointSet(const std::string & filename)
    : root(nullptr)
{
    if (!filename.empty()) {
        std::vector<Point> points = read(filename.c_str());
        balancing(points.begin(), points.end(), Orientation::Vertical);
    }
}

void PointSet::balancing(std::vector<Point>::iterator begin, std::vector<Point>::iterator end, PointSet::Orientation now)
{
    if (begin == end) {
        return;
    }
    std::sort(begin, end, [now](auto p1, auto p2) { return (now == Orientation::Vertical) ? (p1.x() < p2.x()) : (p1.y() < p2.y()); });
    auto middle = begin + (end - begin) / 2;
    put(*middle);
    balancing(begin, middle, next(now));
    balancing(++middle, end, next(now));
}

bool PointSet::empty() const
{
    return root == nullptr || (root->size) == 0u;
}

bool PointSet::contains(const Point & key) const
{
    std::shared_ptr<Node> now = root;
    while (now != nullptr) {
        if (now->point == key) {
            return true;
        }
        if ((now->orientation == Orientation::Vertical && key.x() >= now->point.x()) ||
            (now->orientation == Orientation::Horizontal && key.y() >= now->point.y())) {
            now = now->right;
        }
        else {
            now = now->left;
        }
    }
    return false;
}

std::size_t PointSet::size() const
{
    return (root ? root->size : 0u);
}

void PointSet::put(const Point & key)
{
    Orientation now_orientation = Orientation::Vertical;
    std::shared_ptr<Node> now, prev = nullptr;
    now = root;        // NOLINT
    bool is_now_right; // false - left, true - right
    while (now != nullptr) {
        if (key == now->point) {
            return;
        }
        prev = now;
        prev->size++;
        if ((now->orientation == Orientation::Vertical && key.x() >= now->point.x()) ||
            (now->orientation == Orientation::Horizontal && key.y() >= now->point.y())) {
            is_now_right = true;
            now = now->right;
        }
        else {
            is_now_right = false;
            now = now->left;
        }
        now_orientation = next(now_orientation);
    }
    now = std::make_shared<Node>(key, now_orientation);
    if (prev) {
        if (is_now_right) {
            prev->right = now;
        }
        else {
            prev->left = now;
        }
    }
    else {
        root = now; // NOLINT
    }
}

void PointSet::print(std::ostream & out, const std::shared_ptr<Node> & node)
{
    if (!node) {
        return;
    }
    print(out, node->left);
    out << '\t' << node->point << ",\n";
    print(out, node->right);
}

std::ostream & operator<<(std::ostream & out, const PointSet & set)
{
    out << "PointSet {\n";
    PointSet::print(out, set.root);
    out << "}";
    return out;
}

std::shared_ptr<PointSet::Node> PointSet::save_tree(const std::set<Point> & ans_set) const
{
    std::shared_ptr<Node> ans_root = std::make_shared<Node>(*ans_set.begin(), Orientation::Vertical);
    std::shared_ptr<Node> now = ans_root;
    for (auto it = ++ans_set.begin(); it != ans_set.end(); it++) {
        now->left = std::make_shared<Node>(*it, Orientation::Vertical);
        now = now->left;
    }
    //quarries.push_back(ans_root);
    return ans_root;
}

std::pair<Rect, Rect> PointSet::split(const Rect & rect_now, const Point & node_point, Orientation node_orientation) const
{
    Rect rect_left, rect_right;
    if (node_orientation == Orientation::Vertical) {
        double x = node_point.x();
        rect_right = Rect(Point(x, rect_now.ymin()), rect_now.right_top());
        rect_left = Rect(rect_now.left_bottom(), Point(x, rect_now.ymax()));
    }
    else {
        double y = node_point.y();
        rect_left = Rect(rect_now.left_bottom(), Point(rect_now.xmax(), y));
        rect_right = Rect(Point(rect_now.xmin(), y), rect_now.right_top());
    }
    return {rect_left, rect_right};
}

std::pair<iterator, iterator> PointSet::range(const Rect & key) const
{
    std::set<Point> ans_set;
    Rect rect_now = Rect(Point(-INF, -INF), Point(INF, INF));
    range_impl(key, root, ans_set, rect_now);
    if (ans_set.empty()) {
        return {};
    }
    return {save_tree(ans_set), {}};
}

void PointSet::range_impl(const Rect & key, const std::shared_ptr<Node> & node, std::set<Point> & ans_set, const Rect & rect_now) const
{
    if (node == nullptr) {
        return;
    }
    if (!key.intersects(rect_now)) {
        return;
    }
    if (key.contains(node->point)) {
        ans_set.insert(node->point);
    }
    auto [rect_left, rect_right] = split(rect_now, node->point, node->orientation);
    range_impl(key, node->left, ans_set, rect_left);
    range_impl(key, node->right, ans_set, rect_right);
}

std::optional<Point> PointSet::nearest(const Point & key) const
{
    auto [begin, end] = nearest(key, 1);
    if (begin == end) {
        return {};
    }
    return *begin;
}

std::pair<iterator, iterator> PointSet::nearest(const Point & key, std::size_t k) const
{
    if (k == 0) {
        return {};
    }
    pool::Pool * pool = PoolAllocator<map_node>::create_pool(k + 1);
    std::set<Point> set;
    {
        PoolAllocator<map_node> alloc(*pool);
        point_map ans_set(alloc);
        Rect rect_now = Rect(Point(-INF, -INF), Point(INF, INF));
        nearest_impl(key, k, root, ans_set, rect_now);
        if (ans_set.empty()) {
            return {};
        }
        for (const auto & [dist, point] : ans_set) {
            set.insert(point);
        }
    }
    PoolAllocator<map_node>::destroy_pool(pool);
    return {save_tree(set), {}};
}

void PointSet::nearest_impl(const Point & key, size_t k, const std::shared_ptr<Node> & node, point_map & ans_set, const Rect & rect_now) const
{
    if (node == nullptr) {
        return;
    }
    ans_set.emplace(key.distance(node->point), node->point);
    if (ans_set.size() > k) {
        ans_set.erase(--ans_set.end());
    }
    if (rect_now.distance(key) > (--ans_set.end())->first) {
        return;
    }
    auto [rect_left, rect_right] = split(rect_now, node->point, node->orientation);
    nearest_impl(key, k, node->left, ans_set, rect_left);
    nearest_impl(key, k, node->right, ans_set, rect_right);
}

} // namespace kdtree