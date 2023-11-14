#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <string>
#include <QApplication>
#include <QListWidget>
#include <QKeyEvent>

class CheatNode : public rclcpp::Node
{
    public:
	CheatNode() : Node("CheatNode")
	{
		publisher_=this->create_publisher<std_msgs::msg::String>("cheat", 10);
	}
	void sendKey(char key)
	{
		auto message = std_msgs::msg::String();
		message.data = std::string(1, key);
		RCLCPP_INFO(this->get_logger(), "Publishing: ’%s’", message.data.c_str());
		publisher_->publish(message);
	}
    private:
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class ListWidget : public QListWidget
{
    public:
	    ListWidget(CheatNode *node) : node_(node) {}
    protected:
	    void keyPressEvent(QKeyEvent *event)
	    {
		node_->sendKey(event->text().toStdString()[0]);
	    }
    private:
	    CheatNode *node_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	QApplication app (argc, argv);

	auto node = std::make_shared<CheatNode>();

	ListWidget list(node.get());
	list.setWindowTitle("Cheatcodes");
	list.resize(80, 80);

	list.insertItem(0, "Drücke A --> Doppelte Geschwindigkeit");
	list.insertItem(1, "Drücke U --> Unsichtbar für 2 Sekunden");
	list.insertItem(2, "Drücke L --> Loser Taste: Unsterblich");
	list.insertItem(3, "Alle Angaben ohne Gewähr ;)");
	list.show();

	rclcpp::spin(node);
	rclcpp::shutdown();

	return app.exec();
}
