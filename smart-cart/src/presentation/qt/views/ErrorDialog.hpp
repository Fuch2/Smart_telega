// ===== src/presentation/qt/views/ErrorDialog.hpp =====
#pragma once

#include <QDialog>
#include <QString>

class QLabel;
class QPushButton;

class ErrorDialog final : public QDialog {
    Q_OBJECT
public:
    explicit ErrorDialog(const QString& title,
                         const QString& message,
                         QWidget*       parent = nullptr);

    static void show(const QString& title,
                     const QString& message,
                     QWidget*       parent = nullptr);
};
