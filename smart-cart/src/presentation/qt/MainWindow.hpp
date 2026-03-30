#pragma once
#include <QMainWindow>

class QStackedWidget;
class QPushButton;
class WorkerView;
class AdminView;
class AdminViewModel;

class MainWindow final : public QMainWindow {
    Q_OBJECT
public:
    // ViewModel-ы приходят снаружи из AppBootstrap
    explicit MainWindow(AdminViewModel* adminVm,
                        QWidget* parent = nullptr);
    ~MainWindow() override = default;

private:
    QStackedWidget*  stack_{nullptr};
    WorkerView*      workerView_{nullptr};
    AdminView*       adminView_{nullptr};
    QPushButton*     workerBtn_{nullptr};
    QPushButton*     adminBtn_{nullptr};

    void buildUi(AdminViewModel* adminVm);
    void wireSignals();
};
