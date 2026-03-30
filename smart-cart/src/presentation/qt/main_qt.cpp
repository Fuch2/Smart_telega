#include <QApplication>
#include <iostream>

#include "smartcart_app/AppBootstrap.hpp"

#include "MainWindow.hpp"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("SmartCart");
    app.setApplicationVersion("1.0.0");

    try {
        smartcart::application::AppBootstrap bootstrap(
            CONFIG_DIR "/appsettings.json");

        MainWindow* window = bootstrap.createMainWindow();
        window->show();

        return app.exec();                  // bootstrap жив до конца exec()

    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << std::endl;
        return 1;
    }
}
