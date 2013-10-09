#include "ExecTest.h"

namespace FW {

ExecTest::ExecTest(std::string id) : Visualizer(id) {
}

ExecTest::~ExecTest() {
}

void ExecTest::init() {
	addProperties();
	registerEvents();
}

void ExecTest::render() {
}

void ExecTest::addProperties() {
	gui()->properties()->add<Button>("Do Shit", "doShit")->setCallback( [&] () {
		execute([&] (IO::AbstractProgressBar::Ptr bar) {
			for (unsigned int i=1; i<=10; ++i) {
				bar->poll(i, 10);
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}, [&] () {
			gui()->status()->set("Done");
		}, "Task 1");
	});

	gui()->properties()->add<Button>("Do Pool Shit", "doPoolShit")->setCallback( [&] () {
		/*
		std::function<void (IO::AbstractProgressBar::Ptr bar)> task = [&] (IO::AbstractProgressBar::Ptr bar) {
			for (unsigned int i=1; i<=100; ++i) {
				bar->poll(i, 100);
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
		};
		for (unsigned int i=0; i<4; ++i) {
			execute(task, std::function<void ()>(), "Task "+lexical_cast<std::string>(i));
		}
		*/
		execute([&] (IO::AbstractProgressBarPool::Ptr pool) {
			std::function<void (IO::AbstractProgressBar::Ptr bar)> task = [&] (IO::AbstractProgressBar::Ptr bar) {
				for (unsigned int i=1; i<=10; ++i) {
					bar->poll(i, 10);
					std::this_thread::sleep_for(std::chrono::seconds(1));
				}
			};
			std::vector<std::shared_ptr<std::thread>> threads(4);
			for (unsigned int i=0; i<4; ++i) {
				auto bar = pool->create("Task "+lexical_cast<std::string>(i));
				threads[i] = std::shared_ptr<std::thread>(new std::thread(task, bar));
			}
			for (unsigned int i = 0; i < 4; ++i) {
				threads[i]->join();
			}
			threads.clear();
		}, [&] () {
			gui()->status()->set("Done");
		});
	});
}

void ExecTest::registerEvents() {
}

ExecTest::Factory::Factory() : FW::Factory() {
}

ExecTest::Factory::~Factory() {
}

void ExecTest::Factory::init() {
}

Visualizer::Ptr ExecTest::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
	ExecTest::Ptr vis(new ExecTest(name));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}

} // FW

