#pragma once
#include <QWidget>

class QTableWidget;
class QLineEdit;
class QSpinBox;
class QComboBox;
class QPushButton;
class QLabel;
class QTextEdit;
class AdminViewModel;

class AdminView final : public QWidget {
    Q_OBJECT
public:
    explicit AdminView(AdminViewModel* vm, QWidget* parent = nullptr);
    ~AdminView() override = default;

private:
    AdminViewModel* vm_{nullptr};

    QTableWidget* table_{nullptr};
    QLineEdit* serialEdit_{nullptr};
    QSpinBox* slotsSpin_{nullptr};
    QLineEdit* fwEdit_{nullptr};
    QComboBox* statusCombo_{nullptr};
    QLabel* msgLabel_{nullptr};

    QPushButton* addBtn_{nullptr};
    QPushButton* updateBtn_{nullptr};
    QPushButton* deleteBtn_{nullptr};
    QPushButton* reloadBtn_{nullptr};
    QPushButton* refreshDiagnosticsBtn_{nullptr};
    QPushButton* resetCartBtn_{nullptr};
    QTextEdit* diagnosticsText_{nullptr};

    void resetForm();
    void selectRowById(int id);


    int selectedId_{0};

    void buildUi();
    void bindVm();
    void refreshTable();
    void loadRowToForm(int row);
};
