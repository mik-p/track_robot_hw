#include "track_robot_hw/rest_client.h"

#include <curl/curl.h>


namespace track_robot_hw
{


size_t HWRESTClient::data_callback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

bool HWRESTClient::get(std::string url, std::string& data)
{
  CURL *curl;
  CURLcode res;

  curl = curl_easy_init();

  if(curl)
  {
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, HWRESTClient::data_callback); // return callback
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &data); // return data

    res = curl_easy_perform(curl); // Perform the request, res will get the return code

    if(res != CURLE_OK) // Check for errors
    {
      ROS_ERROR("curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
      return 0;
    }

    // fill data or do something

    curl_easy_cleanup(curl); // always cleanup

    return 1;
  }
  return 0;
}

bool HWRESTClient::post(std::string url, std::string post_fields)
{
  CURL *curl;
  CURLcode res;

  curl = curl_easy_init();

  if(curl)
  {
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_fields.c_str()); // set post

    res = curl_easy_perform(curl); // Perform the request, res will get the return code

    if(res != CURLE_OK) // Check for errors
    {
      ROS_ERROR("curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
      return 0;
    }

    curl_easy_cleanup(curl); // always cleanup

    return 1;
  }
  return 0;
}

} // namespace track_robot_hw
