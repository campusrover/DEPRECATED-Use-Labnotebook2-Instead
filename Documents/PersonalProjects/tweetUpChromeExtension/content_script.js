// Select the first tweet on the page
// const tweet = document.querySelector('[data-testid="tweet"]');

// Change the text of the tweet
// tweet.textContent = "Hello, Twitter!";
console.log("Hello from content_script!")
// console.log(window.location.toString())
// chrome.tabs.query({active: true, currentWindow: true}, function(tabs) {
//   var url = tabs[0].url;
//   console.log(`This is the current url we are in: ${url}`);
// });

let url = document.location.href //Doesn't update well enough. Good solution for now.
const tweet_id = url.split("status/").pop();
console.log(`This is the website url: ${tweet_id}`);

async function fetchResult(tweet_id) {
  try {
    const response = await fetch(`http://localhost:8000/tweetReplier/${tweet_id}`);
    const data = await response.json();
    return data.result;
  } catch (error) {
    console.error(error);
  }
}

async function main(tweet_id) {
  const result = await fetchResult(tweet_id);
  console.log(`The result from the main function${result}`);
}
// fetch(`http://localhost:8000/tweetReplier/${tweet_id}`)
//   .then(response => response.json())
//   .then(data => console.log(data.result));

main(tweet_id)