from django.test import TestCase

# Create your tests here.
from django.urls import reverse

class QuestionDetailViewTests(TestCase):
    def test_future_question(self):
        """
        The detail view of a question with a pub_date in the future
        returns a 404 not found.
        """
        question = ["1", "2","3"]
        # url1= reverse('vote')
        # print(url1)
        d = {
            'question': question,
            'error_message': "You didn't select a choice.",
        }
        d=""
        url = reverse('vote', args=(d,))

        # print (url)
        response = self.client.get(url)
        self.assertEqual(response.status_code, 200)
