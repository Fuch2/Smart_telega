// ===== src/presentation/qt/views/WorkerView.hpp =====
// Исправлено:
//   - include paths относительные
//   - убраны лишние Qt-заголовки из .hpp (перенесены в .cpp)
#pragma once

#include "viewmodels/WorkerViewModel.hpp"
#include "widgets/SlotGridWidget.hpp"

#include <QWidget>

class QLabel;
class QLineEdit;
class QPushButton;

class WorkerView : public QWidget {
    Q_OBJECT

public:
    explicit WorkerView(WorkerViewModel& viewModel, QWidget* parent = nullptr);

private slots:
    void onSlotsUpdated(QVector<SlotCellData> slots);
    void onOperationStateChanged(const QString& state, const QString& message);
    void onErrorOccurred(const QString& message);
    void onBarcodeSubmitted();
    void onCancelClicked();

private:
    WorkerViewModel& viewModel_;

    SlotGridWidget* slotGrid_     = nullptr;
    QLabel*         stateLabel_   = nullptr;
    QLabel*         messageLabel_ = nullptr;
    QLabel*         errorLabel_   = nullptr;
    QLineEdit*      barcodeEdit_  = nullptr;
    QPushButton*    scanButton_   = nullptr;
    QPushButton*    cancelButton_ = nullptr;

    void setupUi();
    void connectSignals();
};
