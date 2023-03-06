
from tweet_replier import main_reply_func
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Set the allowed origins
    allow_credentials=True,
    allow_methods=["*"],  # Set the allowed HTTP methods
    allow_headers=["*"],  # Set the allowed headers
)

# print("Heyyyyy!")
# print(main_reply_func (1631740580890853376,extra_context="Be witty and smart", desired_suggestion_count=5))
@app.get("/tweetReplier/{tweetID}")
def tweetReplier(tweetID: int):
    # print('test test')
    # print(f'This is the params{tweetID}')
    result = main_reply_func (tweetID,extra_context="Be witty and smart", desired_suggestion_count=3)
    return {"result": result}

