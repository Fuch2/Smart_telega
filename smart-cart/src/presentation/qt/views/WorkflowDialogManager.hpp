#pragma once

#include <QObject>
#include <QString>

class QWidget;
class WorkerViewModel;

class WorkflowDialogManager : public QObject {
    Q_OBJECT

public:
    explicit WorkflowDialogManager(QWidget* parent, WorkerViewModel& viewModel);

    void setModalPopupsReady(bool ready);
    void setModuleOnline(bool online);

    void showStagePopupIfNeeded(const QString& stateKey);
    void showModuleStatePopupIfNeeded(const QString& stateKey);
    bool askCartWasAlreadyInWork();

private:
    QWidget* parent_;
    WorkerViewModel& viewModel_;
    bool modalPopupsReady_{false};
    bool moduleOnline_{true};
    QString lastStagePopupKey_;
    QString lastModuleStatePopupKey_;

    static QString stagePopupTitle(const QString& stateKey);
    static QString moduleStatePopupTitle(const QString& stateKey);
    void showLargeStageDialog(const QString& title);
    void showModuleStateDialog(const QString& title, const QString& body);
};
