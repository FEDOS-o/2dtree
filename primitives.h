#pragma once

#include "mpool.h"

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <utility>
#include <vector>

const double INF = std::numeric_limits<double>::infinity();

namespace {
const double EPS = std::numeric_limits<double>::epsilon();

bool double_equal(double x, double y)
{
    return std::abs(x - y) < EPS;
}
} // namespace

class Point
{
public:
    Point(double x, double y)
        : x_(x)
        , y_(y)
    {
    }

    double x() const
    {
        return x_;
    }

    double y() const
    {
        return y_;
    }

    double distance(const Point & another) const
    {
        return std::hypot(this->x() - another.x(), this->y() - another.y());
    }

    bool operator<(const Point & another) const
    {
        if (double_equal(this->x(), another.x())) {
            if (double_equal(this->y(), another.y())) {
                return false;
            }
            return this->y() < another.y();
        }
        return this->x() < another.x();
    }

    bool operator>(const Point & another) const
    {
        return !this->operator<=(another);
    }

    bool operator<=(const Point & another) const
    {
        return this->operator==(another) || this->operator<(another);
    }

    bool operator>=(const Point & another) const
    {
        return !this->operator<(another);
    }

    bool operator==(const Point & another) const
    {
        return double_equal(this->x(), another.x()) && double_equal(this->y(), another.y());
    }

    bool operator!=(const Point & another) const
    {
        return !this->operator==(another);
    }

    friend std::ostream & operator<<(std::ostream & out, const Point & point)
    {
        out << "Point(" << point.x() << " " << point.y() << ")";
        return out;
    }

private:
    double x_, y_;
};

class Rect
{
public:
    Rect()
        : left_bottom_(0, 0)
        , right_top_(0, 0)
    {
    }

    Rect(const Point & left_bottom, const Point & right_top)
        : left_bottom_(left_bottom)
        , right_top_(right_top)
    {
    }

    Point right_top() const
    {
        return right_top_;
    }

    Point left_bottom() const
    {
        return left_bottom_;
    }

    double xmin() const
    {
        return left_bottom_.x();
    }
    double ymin() const
    {
        return left_bottom_.y();
    }
    double xmax() const
    {
        return right_top_.x();
    }
    double ymax() const
    {
        return right_top_.y();
    }
    double distance(const Point & point) const
    {
        if (contains(point)) {
            return 0;
        }
        std::array<double, 6> array = {
                point.distance({xmin(), ymin()}),
                point.distance({xmin(), ymax()}),
                point.distance({xmax(), ymin()}),
                point.distance({xmax(), ymax()}),
                ((point.x() <= xmax() && point.x() >= xmin()) ? std::min(std::abs(point.y() - ymin()), std::abs(point.y() - ymax())) : INF),
                ((point.y() <= ymax() && point.y() >= ymin()) ? std::min(std::abs(point.x() - xmin()), std::abs(point.x() - xmax())) : INF)};
        return *std::min_element(array.begin(), array.end());
    }

    bool contains(const Point & point) const
    {
        return point.x() <= xmax() && point.x() >= xmin() &&
                point.y() <= ymax() && point.y() >= ymin();
    }

    bool intersects(const Rect & another) const
    {
        return intersects_impl(another) || another.intersects_impl(*this);
    }

private:
    bool intersects_impl(const Rect & another) const
    {
        return contains({another.xmin(), another.ymin()}) ||
                contains({another.xmin(), another.ymax()}) ||
                contains({another.xmax(), another.ymin()}) ||
                contains({another.xmax(), another.ymax()});
    }

    Point left_bottom_, right_top_;
};

using map_node = std::_Rb_tree_node<std::pair<double, Point>>;
using point_map = std::map<double, Point, std::less<double>, PoolAllocator<map_node>>;


} // namespace rbtree

namespace kdtree {

class PointSet
{
    enum class Orientation
    {
        Vertical,
        Horizontal,
    };

    Orientation next(Orientation that)
    {
        if (that == Orientation::Vertical) {
            return Orientation::Horizontal;
        }
        return Orientation::Vertical;
    }

    struct Node
    {
        Point point;
        Orientation orientation;
        std::shared_ptr<Node> left, right;
        size_t size;

        Node(const Point & point_, Orientation orientation_)
            : point(point_)
            , orientation(orientation_)
            , size(1u)
        {
        }
    };

    static void print(std::ostream & out, const std::shared_ptr<Node> & node);

    std::pair<Rect, Rect> split(const Rect & rect_now, const Point & node_point, Orientation node_orientation) const;

    std::shared_ptr<Node> save_tree(const std::set<Point> & points) const;

    void range_impl(const Rect & rect, const std::shared_ptr<Node> & node_now, std::set<Point> & ans_set, const Rect & rect_now) const;

    void nearest_impl(const Point & key, size_t k, const std::shared_ptr<Node> & node_now, point_map & ans_map, const Rect & rect_now) const;

    void balancing(std::vector<Point>::iterator begin, std::vector<Point>::iterator end, Orientation now);

public:
    class iterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = Point;
        using pointer = const Point *;
        using reference = const Point &;

        iterator()
        {
        }

        iterator(const std::shared_ptr<Node> & node)
            : now(node)
        {
        }

        reference operator*() const { return now->point; }
        pointer operator->() const { return &(now->point); }

        // Prefix increment
        iterator & operator++()
        {
            if (now == nullptr) {
                return *this;
            }
            if (now->left) {
                queue.push_back(now->left);
            }
            if (now->right) {
                queue.push_back(now->right);
            }
            if (queue.empty()) {
                now = nullptr;
            }
            else {
                now = queue.front();
                queue.pop_front();
            }
            return *this;
        }

        // Postfix increment
        iterator operator++(int)
        {
            iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        friend bool operator==(const iterator & a, const iterator & b)
        {
            return a.now == b.now;
        };
        friend bool operator!=(const iterator & a, const iterator & b)
        {
            return !(a == b);
        };

    private:
        std::deque<std::shared_ptr<Node>> queue;
        std::shared_ptr<Node> now;
    };

    PointSet(const std::string & filename = {});

    bool empty() const;
    std::size_t size() const;
    void put(const Point &);
    bool contains(const Point &) const;

    std::pair<iterator, iterator> range(const Rect &) const;

    iterator begin() const
    {
        return root;
    }
    iterator end() const
    {
        return {};
    }

    std::optional<Point> nearest(const Point &) const;
    std::pair<iterator, iterator> nearest(const Point &, std::size_t) const;

    friend std::ostream & operator<<(std::ostream &, const PointSet &);

private:
    std::shared_ptr<Node> root;
    //mutable std::vector<std::shared_ptr<Node>> quarries;
};

} // namespace kdtree