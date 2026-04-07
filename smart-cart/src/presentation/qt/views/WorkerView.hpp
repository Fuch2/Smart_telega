#pragma once

#include "presentation/qt/viewmodels/WorkerViewModel.hpp"
#include "presentation/qt/widgets/SlotGridWidget.hpp"  // нужен как виджет

#include <QWidget>

class QLabel;
class QLineEdit;
class QPushButton;
class QTextEdit;

class WorkerView : public QWidget {
    Q_OBJECT

public:
    explicit WorkerView(WorkerViewModel& viewModel,
                        QWidget*         parent = nullptr);

private slots:
    void onSlotsUpdated(QVector<SlotCellData> items);
    void onOperationStateChanged(const QString& state,
                                 const QString& message);
    void onWorkflowUpdated(const QString& workflow,
                           const QString& order,
                           const QString& checklist);
    void onErrorOccurred(const QString& message);
    void onImportOrderClicked();
    void onBarcodeSubmitted();
    void onCancelClicked();
    void onArrivedFeederClicked();
    void onStartFeederPrepClicked();
    void onFeederPrepDoneClicked();
    void onArrivedLineClicked();
    void onStartIssuingClicked();
    void onIssueMaterialClicked();
    void onCompleteIssuingClicked();
    void onInspectLeftoversClicked();
    void onStartReturnClicked();
    void onReturnLeftoverClicked();

private:
    WorkerViewModel& viewModel_;

    SlotGridWidget* slotGrid_     = nullptr;
    QLabel*         stateLabel_   = nullptr;
    QLabel*         messageLabel_ = nullptr;
    QLabel*         workflowLabel_ = nullptr;
    QLabel*         orderLabel_   = nullptr;
    QTextEdit*      checklistText_ = nullptr;
    QLabel*         errorLabel_   = nullptr;
    QLineEdit*      barcodeEdit_  = nullptr;
    QPushButton*    importButton_ = nullptr;
    QPushButton*    scanButton_   = nullptr;
    QPushButton*    cancelButton_ = nullptr;
    QPushButton*    arrivedFeederButton_ = nullptr;
    QPushButton*    startFeederPrepButton_ = nullptr;
    QPushButton*    feederPrepDoneButton_ = nullptr;
    QPushButton*    arrivedLineButton_ = nullptr;
    QPushButton*    startIssuingButton_ = nullptr;
    QPushButton*    issueButton_ = nullptr;
    QPushButton*    completeIssuingButton_ = nullptr;
    QPushButton*    inspectLeftoversButton_ = nullptr;
    QPushButton*    startReturnButton_ = nullptr;
    QPushButton*    returnLeftoverButton_ = nullptr;

    void setupUi();
    void connectSignals();
};
