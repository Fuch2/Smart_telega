#pragma once

#include <QDialog>
#include <QString>
#include <QStringList>

class QListWidget;
class QListWidgetItem;
class QLabel;
class QPushButton;

/// Диалог выбора заказа из директории.
/// Отображает список JSON/XLSX файлов с метаданными.
/// - "Выбрать" — подтверждает выбор из списка
/// - "Обзор..." — открывает стандартный QFileDialog
/// - "Отмена" — закрывает без выбора
class OrderSelectionDialog : public QDialog {
    Q_OBJECT

public:
    explicit OrderSelectionDialog(const QString& ordersDir,
                                  QWidget*       parent = nullptr);

    /// Возвращает полный путь к выбранному файлу, либо пустую строку.
    QString selectedPath() const { return selectedPath_; }

private Q_SLOTS:
    void onItemDoubleClicked(QListWidgetItem* item);
    void onSelectClicked();
    void onBrowseClicked();
    void onRefreshClicked();
    void onSearchTextChanged(const QString& text);

private:
    void buildUi();
    void populateList(const QString& filter = {});
    static QString orderSummary(const QString& filePath);

    QString      ordersDir_;
    QString      selectedPath_;

    QListWidget* listWidget_  = nullptr;
    QLabel*      statusLabel_ = nullptr;
    QPushButton* selectBtn_   = nullptr;
};
