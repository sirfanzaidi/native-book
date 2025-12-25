use axum::{http::StatusCode, Json};
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize)]
pub struct QueryRequest {
    pub query: String,
    pub mode: String,
    pub session_id: Option<String>,
}

#[derive(Debug, Serialize)]
pub struct QueryResponse {
    pub answer: String,
    pub sources: Vec<SourceReference>,
    pub latency_ms: u64,
}

#[derive(Debug, Serialize)]
pub struct SourceReference {
    pub chapter_title: String,
    pub module: String,
    pub source_url: String,
    pub relevance_score: f32,
}

#[derive(Debug, Deserialize)]
pub struct CreateSessionRequest {
    pub selected_text: String,
}

#[derive(Debug, Serialize)]
pub struct CreateSessionResponse {
    pub session_id: String,
    pub expires_at: String,
}

pub async fn query_handler(
    Json(_payload): Json<QueryRequest>,
) -> Result<Json<QueryResponse>, StatusCode> {
    // TODO: Implement query processing
    Err(StatusCode::NOT_IMPLEMENTED)
}

pub async fn create_session_handler(
    Json(_payload): Json<CreateSessionRequest>,
) -> Result<Json<CreateSessionResponse>, StatusCode> {
    // TODO: Implement session creation
    Err(StatusCode::NOT_IMPLEMENTED)
}
