# DB test data (MVP)

## Apply from project root

1. Create schema (migration):
```bash
sqlite3 smartcart.db < src/infrastructure/db/migrations/001_init.sql
