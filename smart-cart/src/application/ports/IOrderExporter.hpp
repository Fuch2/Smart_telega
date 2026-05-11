#pragma once

namespace smartcart::application::ports {

/// Порт для записи результата выполненного заказа во внешний файл.
/// Вызывается WorkflowService после завершения выдачи (completeIssuing).
class IOrderExporter {
public:
    virtual ~IOrderExporter() = default;

    /// Сохраняет статус выполнения заказа (order_id в БД) в файл.
    /// Реализация сама определяет формат и путь.
    virtual void exportOrder(int orderId) = 0;
};

} // namespace smartcart::application::ports
