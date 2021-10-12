
#include <QApplication>
#include <QFrame>
#include <rclcpp/rclcpp.hpp>

class TurtlesimFrame : public QFrame
{
public:
    TurtlesimFrame() : QFrame()
    {
    }

    ~TurtlesimFrame()
    {
    }
};
class TurtleSimSink : public QApplication
{
private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<TurtlesimFrame> frame_ptr;
public:
    TurtleSimSink(int &argc, char **argv)
        : QApplication(argc, argv)
    {
        nh_ = rclcpp::Node::make_shared("turtle_sim_sink");
        frame_ptr = std::make_shared<TurtlesimFrame>();
    }

    int exec()
    {
        frame_ptr->show();
        return QApplication::exec();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    TurtleSimSink app(argc, argv);
    app.exec();
    rclcpp::shutdown();
    return 0;
}
