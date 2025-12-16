# backend/src/database/database.py
import os
from typing import Generator
from sqlmodel import Session, create_engine
from dotenv import load_dotenv
load_dotenv()

# Load environment variables (ensure this is done before calling get_engine)
from dotenv import load_dotenv
dotenv_path = os.path.join(os.path.dirname(__file__), '..', '..', '.env')
load_dotenv(dotenv_path=dotenv_path)

DATABASE_URL = os.getenv("DATABASE_URL")
if not DATABASE_URL:
    raise ValueError("DATABASE_URL not found in environment variables.")

engine = create_engine(DATABASE_URL, echo=True)

def get_session() -> Generator[Session, None, None]:
    """
    Dependency to get a database session.
    """
    with Session(engine) as session:
        yield session

def create_db_and_tables():
    """
    Creates database tables based on SQLModel metadata.
    This should be called at application startup or as part of a migration.
    """
    from .models import SQLModel  # Import SQLModel to access metadata
    SQLModel.metadata.create_all(engine)
