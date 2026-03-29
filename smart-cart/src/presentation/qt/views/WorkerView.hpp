#pragma once
#include <QWidget>

class StatusPanelWidget;
class SlotGridWidget;
class WorkerViewModel;

class WorkerView final : public QWidget {
    Q_OBJECT
public:
    explicit WorkerView(QWidget* parent = nullptr);
    ~WorkerView() override = default;

private:
    StatusPanelWidget* status_{nullptr};
    SlotGridWidget* grid_{nullptr};
    WorkerViewModel* vm_{nullptr};

    void buildUi();
    void bindViewModel();
};
